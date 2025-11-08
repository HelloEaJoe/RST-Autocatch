#!/usr/bin/env python
import time
import threading
import time
from multiprocessing import Process, Queue
from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *
import cv2
from pymavlink import mavutil
from .read_depth import *
from .real_read_yaw import *
from packaged_part.pid_catalog_first.sheer_pid import *
from packaged_part.move.move import *
from packaged_part.move.init import *
from packaged_part.move.arm_disarm import arm, disarm
import threading



# def read_depth_continual(Boolean):
#     # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#
#     msg = master.recv_match()
#     if not msg:
#         print(4)
#     if msg.get_type() == 'AHRS2':
#         # print("\nAs dictionary: %s" % msg.to_dict())
#         print(msg.altitude)
#         # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
#         # print("\nSystem status: %s" % msg.system_status)
#     return msg.altitude
#
#
# print(3)
# if __name__ == '__main__':
#     while 1:
#         read_depth_continual(True)

# 全局只初始化一次 master
_depth_master = None
def read_depth_continual(depth):
    global _depth_master
    if _depth_master is None:
        _depth_master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    # 持续读取直到拿到 AHRS2 消息，最多尝试20次，避免丢帧
    last_val = depth
    for _ in range(20):
        msg = _depth_master.recv_match(blocking=True, timeout=0.05)
        if msg and msg.get_type() == 'AHRS2':
            depth_val = -msg.altitude * 100
            if abs(depth_val) > 0.01:
                print("当前深度：", depth_val)
                return depth_val
            last_val = depth_val
        time.sleep(0.01)
    # 没拿到就返回上次的
    print("未获取到新深度，返回上次值", last_val)
    return last_val

if __name__ == '__main__':
    read_depth_continual(True)