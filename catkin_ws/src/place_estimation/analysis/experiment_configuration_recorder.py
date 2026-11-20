#!/usr/bin/env python3
"""Preserve startup settings and package sources alongside each experiment bag."""
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import uuid


DEFAULT_NAMESPACES = [
    '/prior_distribution_node', '/observation_fusion_node_multi_new',
    '/distribution_visualization_node', '/kl_evaluation_node',
    '/predictive_evaluation_node', '/youbot_camera_trajectory_publisher',
    '/track_P_meta',
]


def bag_filename(output_bag):
    path = Path(output_bag).expanduser().resolve()
    return path if path.suffix == '.bag' else Path(str(path) + '.bag')


def git_value(package_directory, arguments):
    try:
        result = subprocess.run(
            ['git', '-C', str(package_directory)] + arguments,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            universal_newlines=True, timeout=3, check=True,
        )
        return result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def capture_configuration(package_directory, output_bag, get_param, ros_time_ns,
                          namespaces=None):
    package_directory = Path(package_directory)
    files = {}
    errors = []
    patterns = [
        ('', 'CMakeLists.txt'), ('', 'package.xml'),
        ('config', '*.yaml'), ('launch', '*.launch'), ('msg', '*.msg'),
        ('scripts', '*.py'), ('evaluation', '*.py'), ('analysis', '*.py'),
    ]
    for folder, pattern in patterns:
        for path in sorted((package_directory / folder).glob(pattern)):
            try:
                data = path.read_bytes()
                files[str(path.relative_to(package_directory))] = {
                    'sha256': hashlib.sha256(data).hexdigest(),
                    'content': data.decode('utf-8'),
                }
            except (OSError, UnicodeError) as error:
                errors.append('%s: %s' % (path, error))
    status = git_value(package_directory, ['status', '--porcelain', '--untracked-files=no', '--', '.'])
    return {
        'schema_version': 1,
        'recording_id': uuid.uuid4().hex,
        'captured_at_utc': datetime.now(timezone.utc).isoformat(),
        'ros_time_ns': int(ros_time_ns),
        'requested_bag': str(bag_filename(output_bag)),
        'git_revision': git_value(package_directory, ['rev-parse', 'HEAD']),
        'git_tracked_files_modified': None if status is None else bool(status),
        'runtime_parameters': {
            name: copy.deepcopy(get_param(name, None))
            for name in (DEFAULT_NAMESPACES if namespaces is None else namespaces)
        },
        'package_files': files,
        'capture_errors': errors,
        'scope': (
            'Startup ROS parameter-server values plus available place_estimation sources/configs. '
            'Missing namespaces are null; code defaults are in the saved sources. '
            'External Unity/YOLO sources, model weights and runtime performance are not captured.'
        ),
    }


def save_sidecar(record):
    path = Path(record['requested_bag']).with_suffix('.settings.json')
    path.parent.mkdir(parents=True, exist_ok=True)
    # Never replace settings belonging to an earlier experiment.
    with path.open('x', encoding='utf-8') as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
    return path


def main():
    import rospkg
    import rospy
    from std_msgs.msg import String

    rospy.init_node('experiment_configuration_recorder')
    try:
        output_bag = rospy.get_param('~output_bag')
        record = capture_configuration(
            rospkg.RosPack().get_path('place_estimation'), output_bag,
            rospy.get_param, rospy.Time.now().to_nsec(),
            rospy.get_param('~parameter_namespaces', DEFAULT_NAMESPACES),
        )
        payload = json.dumps(record, ensure_ascii=False, sort_keys=True, allow_nan=False)
        publisher = rospy.Publisher('/experiment_configuration', String, queue_size=1, latch=True)
        publisher.publish(String(data=payload))
        try:
            path = save_sidecar(record)
            rospy.loginfo('Experiment settings saved: %s', path)
        except OSError as error:
            # A latched copy remains available to rosbag, including when the
            # sidecar name already exists. Do not overwrite historical settings.
            rospy.logwarn('Could not save settings sidecar: %s; settings remain on /experiment_configuration', error)
        rospy.spin()
    except (KeyError, TypeError, ValueError, rospy.ROSException) as error:
        rospy.logfatal('Experiment configuration capture failed: %s', error)


if __name__ == '__main__':
    main()
