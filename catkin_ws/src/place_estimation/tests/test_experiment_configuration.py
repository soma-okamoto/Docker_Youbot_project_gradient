"""Configuration snapshots retain the settings actually captured at recording."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'configuration_under_test', ROOT / 'analysis/experiment_configuration_recorder.py')
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class ConfigurationRecorderTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / 'config').mkdir()
        (self.root / 'scripts').mkdir()
        (self.root / 'config/prior.yaml').write_text('learning_min_samples: 10\n')
        (self.root / 'scripts/node.py').write_text('print("snapshot")\n')
        self.parameters = {
            '/prior_distribution_node': {'learning_min_samples': 10, 'covariance_initial_weight': .5},
            '/observation_fusion_node_multi_new': {'min_observation_frames': 2, 'observation_timeout': 5.},
        }

    def capture(self, output='trial_10'):
        return recorder.capture_configuration(
            self.root, self.root / output, lambda name, default=None: self.parameters.get(name, default), 123)

    def test_exact_parameters_sources_and_hashes_survive_later_changes(self):
        snapshot = self.capture()
        self.parameters['/prior_distribution_node']['learning_min_samples'] = 2
        (self.root / 'config/prior.yaml').write_text('learning_min_samples: 2\n')
        self.assertEqual(snapshot['runtime_parameters']['/prior_distribution_node']['learning_min_samples'], 10)
        original = snapshot['package_files']['config/prior.yaml']
        self.assertEqual(original['content'], 'learning_min_samples: 10\n')
        self.assertEqual(original['sha256'], hashlib.sha256(original['content'].encode()).hexdigest())
        self.assertIsNone(snapshot['runtime_parameters']['/track_P_meta'])
        self.assertIsNone(snapshot['git_revision'])

    def test_bag_and_prefix_produce_same_sidecar_path(self):
        self.assertEqual(recorder.bag_filename(self.root/'trial_10'), self.root/'trial_10.bag')
        self.assertEqual(recorder.bag_filename(self.root/'trial_10.bag'), self.root/'trial_10.bag')
        snapshot = self.capture('trial_10.bag')
        path = recorder.save_sidecar(snapshot)
        self.assertEqual(path, self.root/'trial_10.settings.json')
        self.assertEqual(json.loads(path.read_text()), snapshot)

    def test_existing_historical_settings_are_never_overwritten(self):
        snapshot = self.capture()
        path = recorder.save_sidecar(snapshot)
        with self.assertRaises(FileExistsError):
            recorder.save_sidecar(self.capture())
        self.assertEqual(json.loads(path.read_text()), snapshot)

    def test_main_publishes_a_latched_snapshot_for_a_later_rosbag_subscriber(self):
        rospy, rospkg, std = ModuleType('rospy'), ModuleType('rospkg'), ModuleType('std_msgs.msg')
        std.String = NS
        settings = {'~output_bag': str(self.root/'trial_10')}
        settings.update(self.parameters)
        rospy.get_param = lambda name, default=None: settings.get(name, default)
        rospy.Time = NS(now=lambda: NS(to_nsec=lambda: 123))
        rospy.ROSException = RuntimeError
        rospy.Publisher = Mock(return_value=Mock())
        for name in ('init_node', 'loginfo', 'logwarn', 'logfatal', 'spin'):
            setattr(rospy, name, Mock())
        rospkg.RosPack = lambda: NS(get_path=lambda name: str(self.root))
        modules = {'rospy': rospy, 'rospkg': rospkg, 'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': std}
        with patch.dict(sys.modules, modules):
            recorder.main()
        self.assertTrue(rospy.Publisher.call_args.kwargs['latch'])
        record = json.loads(rospy.Publisher.return_value.publish.call_args.args[0].data)
        self.assertEqual(record['runtime_parameters']['/prior_distribution_node']['covariance_initial_weight'], .5)
        self.assertEqual(json.loads((self.root/'trial_10.settings.json').read_text()), record)
        rospy.logfatal.assert_not_called()

    def test_experiment_launch_records_configuration_topic_and_starts_capture_node(self):
        launch = ET.parse(ROOT/'launch/preliminary_experiment.launch').getroot()
        nodes = {node.attrib['name']: node for node in launch.findall('node')}
        self.assertIn('/experiment_configuration', nodes['preliminary_experiment_recorder'].attrib['args'])
        self.assertEqual(nodes['experiment_configuration_recorder'].attrib['required'], 'true')


if __name__ == '__main__':
    unittest.main()
