import cv2
import threading
import time
from multiprocessing import Event, Process, Queue
from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *
import cv2
from pymavlink import mavutil
from parameter.read_depth import *
from parameter.real_read_yaw import *
from pid_catalog_first.sheer_pid import *
from move.move import *
from move.init import *
from move.arm_disarm import arm, disarm
depth_queue = Queue(maxsize=1)

def read_depth_continual():
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'AHRS2':
            # depth = -msg.altitude * 10
            if not depth_queue.empty():
                depth_queue.get()
            depth_queue.put(-msg.altitude * 100)

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
disarm()
init()
arm()
    # 启动检测图像和检测深度的进程
depth_thread = Process(target=read_depth_continual)
depth_thread.start()
# 打开摄像头（0表示默认摄像头）
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头！")
    exit()

# 获取摄像头帧尺寸
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_size = (width, height)

# 设置编码器和输出路径
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # AVI格式
output_video = "camera_recording.avi"
fps = 30  # 帧率

# 创建VideoWriter对象
out = cv2.VideoWriter(output_video, fourcc, fps, frame_size)
target_depth=30
print("录制中... 按 'q' 键停止")
while True:
    aim(1300,1500,1500,1500)
    ret, frame = cap.read()  # 读取一帧
    if not depth_queue.empty():
        depth = depth_queue.get()
    if not ret:
        print("无法获取帧，退出")
        break
    pwm1 = int(depth_pid.calculate(target_depth, depth, 1365))    
    out.write(frame)  # 写入视频
    cv2.imshow('Recording', frame)  # 实时预览
    # aim(pwm1,1500,1500,1500)
    # 按 'q' 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
out.release()
cv2.destroyAllWindows()
print(f"录制完成，视频保存至：{output_video}")