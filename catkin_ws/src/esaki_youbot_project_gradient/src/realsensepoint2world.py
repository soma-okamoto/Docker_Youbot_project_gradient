import pyrealsense2 as rs
import numpy as np
import cv2

# カメラの設定
conf = rs.config()
# RGB
conf.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# 距離
conf.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# stream開始
pipe = rs.pipeline()
profile = pipe.start(conf)

cnt = 0

try:
    while True:
        frames = pipe.wait_for_frames()

        # frameデータを取得
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # 画像データに変換
        color_image = np.asanyarray(color_frame.get_data())
        # 距離情報をカラースケール画像に変換する
        depth_color_frame = rs.colorizer().colorize(depth_frame)
        depth_image = np.asanyarray(depth_color_frame.get_data())

        #お好みの画像保存処理
        
finally:
    # Stop streaming
    profile.release()
    cv2.destroyAllWindows()

