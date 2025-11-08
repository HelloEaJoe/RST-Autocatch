# from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *
# from pymavlink import mavutil
# import threading
# import time
# from multiprocessing import Process, Queue
# from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *
import cv2
# from pymavlink import mavutil
# from parameter.read_depth import *
# from parameter.real_read_yaw import *
# from pid_catalog_first.sheer_pid import *
# from move.move import *
# from move.init import *
# from move.arm_disarm import arm, disarm

# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# master.wait_heartbeat()
# disarm()
# init()
# arm()
# 电脑的模型路径
# model_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\weights\best_1.onnx"
# label_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\data\xian.yaml"
# pi的模型路径
# model_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/weights/best_1.onnx"
# label_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/data/xian.yaml"

# net = yolov5_lite(model_path, label_path, confThreshold=0.5, nmsThreshold=0.5)
cap = cv2.VideoCapture(0)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# 得到摄像头拍摄的视频的宽和高
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# 创建对象，用于视频的写出
videoWrite = cv2.VideoWriter('record_in_sence.mp4', fourcc, 30, (width, height))

ret, frame = cap.read()
while ret:
    ret, frame = cap.read()
    # process_frame = net.detect(frame)

    videoWrite.write(frame)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

videoWrite.release()
cap.release()
cv2.destroyAllWindows()

