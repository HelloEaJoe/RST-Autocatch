import queue
import time
import threading
from pymavlink import mavutil
from visual.pipeline import*
#from YOLOv5Litemaster.pipeline_jiao import*
from move.arm_disarm import arm,disarm
from move.move import *
from move.init import *
from pid_catalog.set_deepth import*
from pid_catalog.sheer_pid import *
from YOLOv5Litemaster.detectpipeline import *
#from visual.pipeline import *
from camera_gimbal_control.gimabal_tilt_servo3 import *
from camera_gimbal_control.LED import *

        


if __name__=='__main__':
    print("GO!")
    master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
    master.wait_heartbeat()
    disarm()
    init()
    arm()
    global pwm1,pwm2,pwm3,flag
    pwm1,pwm2,pwm3,flag=1500,1500,1500,0
    
    while True:
        aim(1420,1500,1500) #分别是上浮下潜、左右偏航1493、前进后退
        









