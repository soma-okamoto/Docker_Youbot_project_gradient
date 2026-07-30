#!/usr/bin/env python3
import pyrealsense2 as rs

import argparse
import os
import sys
from pathlib import Path
import numpy as np
import rospy
import torch
import torch.backends.cudnn as cudnn
from std_msgs.msg import Float32MultiArray,Bool
from brics_actuator.msg import JointPositions
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH


import os
# limit the number of cpus used by high performance libraries
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

WEIGHTS = ROOT / 'weights'

if str(ROOT / 'yolov5') not in sys.path:
    sys.path.append(str(ROOT / 'yolov5'))  # add yolov5 ROOT to PATH
if str(ROOT / 'strong_sort') not in sys.path:
    sys.path.append(str(ROOT / 'strong_sort'))  # add strong_sort ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import logging
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.dataloaders import VID_FORMATS, LoadImages, LoadStreams,IMG_FORMATS
from yolov5.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, check_requirements, cv2,
                                  check_imshow, xyxy2xywh, increment_path, strip_optimizer, colorstr, print_args, check_file)
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.plots import Annotator, colors, save_one_box
from strong_sort.utils.parser import get_config
from strong_sort.strong_sort import StrongSORT


from sensor_msgs.msg import Image #12.4new
from cv_bridge import CvBridge, CvBridgeError #12.4new
import message_filters
from sensor_msgs.msg import CameraInfo
from yolov5_ros.msg import yolov5_data


camerainfo = CameraInfo()
color_intr = rs.intrinsics()
fps = 60.
delay = 1/fps*0.5
pubdata = yolov5_data()

# 5関節分の指令値を受信するまで空のメッセージを保持する
cameraPosition = JointPositions()
camera_position_ready = False

def ImageCallback(depthdata, colordata):
    global color_frame
    global depth_frame
    color_frame = colordata
    depth_frame = depthdata

def camerainfo_callback(msg_camerainfo):
    global color_intr
    color_intr.height =  msg_camerainfo.height
    color_intr.width = msg_camerainfo.width
    color_intr.fx = msg_camerainfo.K[0]
    color_intr.fy = msg_camerainfo.K[4]
    color_intr.ppx = msg_camerainfo.K[2]
    color_intr.ppy = msg_camerainfo.K[5]
    color_intr.model = rs.distortion.inverse_brown_conrady
    color_intr.coeffs = msg_camerainfo.D

def bool_callback(bool_msg):
    global goal_status
    goal_status = bool_msg.data
    rospy.loginfo("Received bool data : %s",bool_msg.data)
    

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)


#affine

#eef（グリッパー）から見たobject位置を入力するとbase_footprintに変換される

def DegToRad(th):
    rad=(np.pi/180)*th
    return rad

def RadToDeg(rad):
    deg = rad *(180/np.pi)
    return deg

# 回転行列
def rot_x(th):
  
  c = np.cos(th)
  s = np.sin(th)
  
  Rx = np.array([[1, 0, 0],
               [0, c, -s],
               [0, s, c]])
  
  return Rx

def rot_y(th):
  
  c = np.cos(th)
  s = np.sin(th)
  
  Ry = np.array([[c, 0, s],
               [0, 1, 0],
               [-s, 0, c]])

  return Ry

def rot_z(th):
  
  c = np.cos(th)
  s = np.sin(th)
  
  Rz = np.array([[c, -s, 0],
               [s, c, 0],
               [0, 0, 1]])  
  
  return Rz

def CoordinateTransformation(x,y,z,color_intr):
    global cameraPosition, camera_position_ready

    # position_command未受信時に positions[0] へアクセスしない
    if not camera_position_ready or len(cameraPosition.positions) < 5:
        rospy.logwarn_throttle(2.0,
            "Joint positions are not ready; skipping coordinate transformation")
        return None

    if not np.isfinite(z) or z <= 0.0:
        rospy.logwarn_throttle(2.0,
            "Invalid depth value received: %s; skipping detection", str(z))
        return None

    # コールバック更新との競合を避けるため、必要な値だけローカルへコピー
    joint_values = [p.value for p in cameraPosition.positions[:5]]

    world_point = rs.rs2_deproject_pixel_to_point(color_intr , [x,y],z)
    from_eef_to_object=np.array([[-world_point[1]],[world_point[0]],[z]])
    #アームに司令されているもの
    # q1 = -DegToRad(-60)+DegToRad(169)
    # q2 = DegToRad(70)+DegToRad(65)
    # q3 = DegToRad(-110)-DegToRad(146)
    # q4 = DegToRad(-60)+DegToRad(102.5)
    # q5 = -DegToRad(0)+DegToRad(167.5)

    test1 = -(joint_values[0] - DegToRad(169))
    test2 = joint_values[1] - DegToRad(65)
    test3 = joint_values[2] + DegToRad(146)
    test4 = joint_values[3] - DegToRad(102.5)
    test5 = -(joint_values[4] - DegToRad(167.5))
    #print("DegToRad:::::" + str(DegToRad(q3)))
    #print("test :::::" + str(test3))

    rot_camera=rot_y(DegToRad(-8))
    trans5=np.array([[0.055],[-0.0332],[0.06]])
    rot_5=rot_z(test5)

    trans4=np.array([[0],[0],[0.13]])
    rot_4=rot_y(test4)

    trans3=np.array([[0],[0],[0.135]])
    rot_3=rot_y(test3)

    trans2=np.array([[0],[0],[0.155]])
    #rot_2=rot_y(DegToRad(q2))
    rot_2=rot_y(test2)

    trans1=np.array([[0.033],[0],[0.019]])
    rot_1=rot_z(test1)
    #0.091は地面からbase_linkへの並進移動
    # trans0=np.array([[-0.147],[0],[0.152+0.091]])
    # trans0=np.array([[-0.147],[0],[0.152]])
    # trans0=np.array([[-0.147],[0],[-0.173]])#arm_origin
    trans0=np.array([[-0.147],[0],[-0.0173]])#arm_origin
    
    # trans0 = np.array([[-0.123], [0], [0.051]])#arm_link0
    rot_0=rot_z(DegToRad(180))

    #baseから見たobject
    object2base=trans0+rot_0@(rot_1@(trans1+rot_2@(trans2+rot_3@(trans3+rot_4@(trans4+rot_5@(rot_camera@(trans5+from_eef_to_object)))))))

    return object2base


#Youbot_2_Position_get

def position_callback(msg):
    global cameraPosition, camera_position_ready

    if len(msg.positions) < 5:
        camera_position_ready = False
        rospy.logwarn_throttle(2.0,
            "Waiting for 5 joint positions from /arm_2/arm_controller/position_command "
            "(received: %d)", len(msg.positions))
        return

    cameraPosition = msg
    camera_position_ready = True

import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Point,PointStamped


@torch.no_grad()
def run(
        weights=ROOT / 'best2.pt',  # model.pt path(s)
        source=ROOT / 'data/images',  # file/dir/URL/glob, 0 for webcam
        strong_sort_weights=WEIGHTS / 'osnet_x0_25_msmt17.pt',
        config_strongsort=ROOT / 'strong_sort/configs/strong_sort.yaml',
        data=ROOT / 'data/data.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.63,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        hide_class=False,
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
):


    try:

        global color_frame
        global depth_frame
        global color_intr
        global depth_image
        global color_image
        global goal_status

        bridge = CvBridge()

        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        mf = message_filters.ApproximateTimeSynchronizer([depth_sub, color_sub], 10, delay)
        mf.registerCallback(ImageCallback)

        camerainfo_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, camerainfo_callback)

        goal_sub = rospy.Subscriber('goal_Pub',Bool,bool_callback)

        goal_status = Bool()
        
        source = str(source)
        save_img = not nosave and not source.endswith('.txt')  # save inference images
        is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
        is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
        webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
        if is_url and is_file:
            source = check_file(source)  # download

        # Directories
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
        (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Load model
        device = select_device(device)
        model = DetectMultiBackend(weights=os.path.join(os.getcwd(),'best2.pt'), device=device, dnn=dnn, data=data, fp16=half)
        stride, names, pt = model.stride, model.names, model.pt
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Dataloader
        webcam= False
        if webcam:
            view_img = check_imshow()
            cudnn.benchmark = True  # set True to speed up constant image size inference
            dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
            
            bs = len(dataset)  # batch_size
        else:
            dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
            bs = 1  # batch_size
        vid_path, vid_writer = [None] * bs, [None] * bs



        cfg = get_config()
        cfg.merge_from_file(opt.config_strongsort)

        # Create as many strong sort instances as there are video sources
        strongsort_list = []
        for i in range(bs):
            strongsort_list.append(
                StrongSORT(
                    strong_sort_weights,
                    device,
                    max_dist=cfg.STRONGSORT.MAX_DIST,
                    max_iou_distance=cfg.STRONGSORT.MAX_IOU_DISTANCE,
                    max_age=cfg.STRONGSORT.MAX_AGE,
                    n_init=cfg.STRONGSORT.N_INIT,
                    nn_budget=cfg.STRONGSORT.NN_BUDGET,
                    mc_lambda=cfg.STRONGSORT.MC_LAMBDA,
                    ema_alpha=cfg.STRONGSORT.EMA_ALPHA,

                )
            )
        outputs = [None] * bs

        # Run inference
        model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup

  

    
        dt, seen = [0.0, 0.0, 0.0, 0.0], 0
        curr_frames, prev_frames = [None] * bs, [None] * bs

        pub = rospy.Publisher('/multi_command', Float32MultiArray, queue_size=10)
        box_pub = rospy.Publisher('/box_command',Float32MultiArray,queue_size=10)
        pub_box = rospy.Publisher("/multi_box_command",yolov5_data,queue_size=10)
        # precameraPosition = 0
        # t_start = rospy.get_time()
        # time_status = False
        while not rospy.is_shutdown():
            if not getattr(color_frame, "encoding", "") or not getattr(depth_frame, "encoding", ""):
                rospy.sleep(0.01)
                continue
            try:
                color_image = bridge.imgmsg_to_cv2(color_frame, 'bgr8')
                depth_image = bridge.imgmsg_to_cv2(depth_frame, '32FC1')


            except CvBridgeError as e:
                rospy.logerr(f"cv_bridge failed: {e}")
                continue
                
            pubdata.header.stamp = color_frame.header.stamp
            
            time = rospy.get_time()



            for frame_idx, (path, im, im0s, vid_cap, s) in enumerate(dataset):
            # for path, im, im0s, vid_cap, s in dataset:
                save_img = True
                path = os.getcwd()
                #ret,frames = cap.read()
                #if not depth_image: continue
                depth = depth_image

                im0s = color_image
                depth_copy = depth.copy()
                img = letterbox(im0s)[0]
                img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
                im = np.ascontiguousarray(img)
                t1 = time_sync()
                im = torch.from_numpy(im).to(device)
                im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
                im /= 255  # 0 - 255 to 0.0 - 1.0
                if len(im.shape) == 3:
                    im = im[None]  # expand for batch dim
                t2 = time_sync()
                dt[0] += t2 - t1

                # Inference
                visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
                pred = model(im, augment=augment, visualize=visualize)
                t3 = time_sync()
                dt[1] += t3 - t2

                # NMS
                pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
                dt[2] += time_sync() - t3

                # Second-stage classifier (optional)
                # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

                # Process predictions
                for i, det in enumerate(pred):  # per image

                    seen += 1
                    if webcam:  # batch_size >= 1
                        p, im0, frame = path[i], im0s[i].copy(), dataset.count
                        s += f'{i}: '
                    else:
                        p, s, im0 = path, '', im0s.copy()
                        # p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)
                    curr_frames[i] = im0

                    p = Path(p)  # to Path
                    
                    annotator = Annotator(im0, line_width=2, pil=not ascii)
                    if cfg.STRONGSORT.ECC:  # camera motion compensation
                        strongsort_list[i].tracker.camera_update(prev_frames[i], curr_frames[i])

                    if det is not None and len(det):
                        # Rescale boxes from img_size to im0 size
                        det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                        xywhs = xyxy2xywh(det[:, 0:4])
                        confs = det[:, 4]
                        clss = det[:, 5]
                        #print("xywhs :::::::: " + str(xywhs))
                        outputs[i] = strongsort_list[i].update(xywhs.cpu(), confs.cpu(), clss.cpu(), im0)
                        detect = []
                        detect_box = []
                        boxs = []
                        if len(outputs[i]) > 0:
                            
                            for j, (output, conf) in enumerate(zip(outputs[i], confs)):
                                bboxes = output[0:4]
                                id = output[4]
                                cls = output[5]
                                #print(" test ::::" + str(bboxes))
                                if(float(conf) >= 0.5):
                                    
                                    #print("conf ::::::::::: " + str(conf))
                                    if save_img or save_crop or view_img:  # Add bbox to image
                                        c = int(cls)  # integer class
                                        
                                        d1, d2 = int((int(output[0])+int(output[2]))/2), int((int(output[1])+int(output[3]))/2)
                                        #端のオブジェクトを検出しない
                                        if(d1 > 50 and d1 < 590):
                                            # print(d1)
                                            #print("d1 d2 ::::::::::" + str(d1) +  "::::::::::: " + str(d2))
                                            zDepth = depth[int(d2),int(d1)]


                                            x = output[0] + (output[2] - output[0])/2
                                            y = output[1] + (output[3] - output[1])/2

                                            object2base = CoordinateTransformation(x, y, zDepth/1000, color_intr)
                                            if object2base is None:
                                                continue

                                            world_point = rs.rs2_deproject_pixel_to_point(
                                                color_intr, [x, y], zDepth/1000)


                                                                                
                                            label = None if hide_labels else (f'{id} {names[c]}' if hide_conf else \
                                            (f'{id} {conf:.2f}' if hide_class else f'{id} {c} {float(object2base[0][0]):.2f} {float(object2base[1][0]):.2f} {float(object2base[2][0]):.2f}'))




                                            detect.append(float(object2base[0][0]))
                                            detect.append(float(object2base[1][0]))
                                            detect.append(float(object2base[2][0]))
                                            detect.append(float(c))
                                            detect.append(float(id))

                                            annotator.box_label(bboxes, label, color=colors(c, True))

                            array_forPublish = Float32MultiArray(data=detect)
                            pub.publish(array_forPublish)                


                    else:
                        pub_box.publish(pubdata)        

                        strongsort_list[i].increment_ages()
                        LOGGER.info('No detections')
                    
                    # Write results
                    im0 = annotator.result()
                    cv2.imshow(str(p), im0)
                    #cv2.imshow("depth_img",depth_copy)
                    cv2.waitKey(1)  # 1 millisecond
                    prev_frames[i] = curr_frames[i]
                
                LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        #cap.release()
        #pipeline.release()
        cv2.destroyAllWindows()

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    #LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if update:
        strip_optimizer(weights)  # update model (to fix SourceChangeWarning)


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'best2.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default=ROOT / 'yolov5/data/images', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--strong-sort-weights', type=str, default=WEIGHTS / 'osnet_x0_25_msmt17.pt')
    parser.add_argument('--config-strongsort', type=str, default='strong_sort/configs/strong_sort.yaml')
    parser.add_argument('--data', type=str, default=ROOT / 'data/data.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--hide-class', default=False, action='store_true', help='hide IDs')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    # print_args(vars(opt))
    return opt


def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))
    # Print results
    # if seen > 0:
    #     t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    # else:
    #     t = (0.0, 0.0, 0.0, 0.0)



if __name__ == "__main__":
    rospy.init_node("ObjectDetection", anonymous=True)
    #rate = rospy.Rate(10)
    color_frame = Image()
    depth_frame = Image()
    color_intr = rs.intrinsics()


    # cameraPosition は購読開始前に初期化済み。購読後に空メッセージで上書きしない。
    cameraPosition_sub = rospy.Subscriber(
        "/arm_2/arm_controller/position_command",
        JointPositions,
        position_callback,
        queue_size=1
    )
    rospy.loginfo("Waiting for 5 joint positions from /arm_2/arm_controller/position_command")

    opt = parse_opt()
    main(opt)