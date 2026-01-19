#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

class RealsenseCapture:

    def __init__(self):
        self.WIDTH = 640
        self.HEGIHT = 480
        self.FPS = 30
        # Configure depth and color streams
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEGIHT, rs.format.bgr8, self.FPS)
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEGIHT, rs.format.z16, self.FPS)

    def start(self):
        # Start streaming
        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)
        print('pipline start')
        # Alignオブジェクト生成
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def read(self, is_array=True):
        # Flag capture available
        ret = True
        # get frames
        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)

        # separate RGB and Depth image
        self.color_frame = aligned_frames.get_color_frame()  # RGB
        self.depth_frame = aligned_frames.get_depth_frame()  # Depth

        if not self.color_frame or not self.depth_frame:
            ret = False
            return ret, (None, None)
        elif is_array:
            # Convert images to numpy arrays
            color_image = np.array(self.color_frame.get_data())
            depth_image = np.array(self.depth_frame.get_data())
            return ret, (color_image, depth_image)
        else:
            return ret, (self.color_frame, self.depth_frame)

    def release(self):
        # Stop streaming
        self.pipeline.stop()



cap = RealsenseCapture()
# プロパティの設定
cap.WIDTH = 640
cap.HEIGHT = 480
cap.FPS = 30
# cv2.VideoCapture()と違ってcap.start()を忘れずに
cap.start()

while True:
    ret, frames = cap.read()  # frames[0]にRGB、frames[1]にDepthの画像がndarrayが入っている
    color_frame = frames[0]
    depth_frame = frames[1]
    # ヒートマップに変換
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        depth_frame, alpha=0.08), cv2.COLORMAP_JET)
    # レンダリング
    images = np.hstack((color_frame, depth_colormap))  # RGBとDepthを横に並べて表示
    cv2.imshow('RealSense', images)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ストリーミング停止
cap.release()
cv2.destroyAllWindows()