#!/usr/bin/env python
import time
from pymavlink import mavutil








def read_yaw_continually(Boolean):
    # 创建 MAVLink 连接
    master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)

    # 等待连接
    # master.wait_heartbeat()
    while Boolean:
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if not msg:
            print('no_msg_y')
            continue
        if msg.get_type() == 'ATTITUDE':
            yaw = msg.yaw
            yaw_degrees = yaw * 180 / 3.14159
            yaw_return =  ( yaw_degrees / 360 ) +0.5
            # print("\nAs dictionary: %s" % msg.to_dict())
            print("当前航向：",  yaw_degrees)
            return yaw_return






    # try:
    #     while True:
    #         # 读取一条消息
    #         msg = master.recv_match(type='ATTITUDE', blocking=True)
    #         if msg is not None:
    #             # 打印航向信息
    #             yaw = msg.yaw  # 这里的 yaw 可能需要转换为度数
    #             print(f"Yaw: {yaw}")
    # except KeyboardInterrupt:
    #     print("程序被用户中断")

if __name__ == '__main__':
    read_yaw_continually()
