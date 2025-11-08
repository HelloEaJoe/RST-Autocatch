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

def make_print_to_file(path='./'):
    '''
    path， it is a path for save your log about fuction print
    example:
    use  make_print_to_file()   and the   all the information of funtion print , will be write in to a log file
    :return:
    '''
    import sys
    import os
    import sys
    import datetime
 
    class Logger(object):
        def __init__(self, filename="Default.log", path="./"):
            self.terminal = sys.stdout
            self.path= os.path.join(path, filename)
            self.log = open(self.path, "a", encoding='utf8',)
            print("save:", os.path.join(self.path, filename))
 
        def write(self, message):
            self.terminal.write(message)
            self.log.write(message)
 
        def flush(self):
            pass

    fileName = datetime.datetime.now().strftime('day'+'%Y_%m_%d')
    sys.stdout = Logger(fileName + '.log', path=path)
 
    #############################################################
    # 这里输出之后的所有的输出的print 内容即将写入日志
    #############################################################
    print(fileName.center(60,'*'))

if __name__=='__main__':
    make_print_to_file(path='./')
    print("GO!")
    master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
    master.wait_heartbeat()
    disarm()
    init()
    arm()
    state_now = 'pipline_light'
    tube_center_last = 0 
    global pwm1,pwm2,pwm3,flag,mean
    pwm1,pwm2,pwm3,flag,mean=1500,1500,1500,0,23
    net = yolov5_lite("cube_best.onnx", "cube.names", 0.25, 0.3)
    camera = cv2.VideoCapture(0)
    time_start = 0
    time_over = 0
    count = 0
    m = means = 0
    cube = False
    cube_last = False
    angle = False
    angle_last = False
    cube_name = None
    cube_name_last = None
    cube_name_final = None
    angle_point = None
    angle_lastpoint = None
    open_claw()
    while True:
        print(state_now)
        #print(state_now)
        ret, frame = camera.read()
        angle_last = angle
        cube_last = cube
        cube_name_last = cube_name
        if ret:
            frame = cv2.resize(frame, (320, 320))
            frame2 = frame.copy()
            count = (count + 1) % 3

            if count == 0:
                frame2, angle, angle_point, cube, cube_name = net.detect(frame2, 100, 130)
                #print(angle, angle_point, cube, cube_name)
                
                angle_lastpoint = angle_point
            else:
                cube_name = cube_name_last
                angle_point = angle_lastpoint
            m, means = patrol(frame, True)
            cv2.imshow("img2", frame2)
            cv2.waitKey(1)
        

        time_over = time.time()
        if (time_over-time_start) > 0.7:
            open_claw()
            

        tube_center = q_center.get()
        if state_now == 'pipline_light':
            if tube_center is not None: #patrol
                center = (tube_center[0])/320
                tube_center_last = center
                print(center)
                if angle == False:
                    pwm2 = int(yaw_pid_partrol_pipline_light_fast.calculate(0.5,center,1495))
                else:
                    pwm2 = int(yaw_pid_partrol_pipline_light_slow.calculate(0.5,center,1495))
                print(pwm2)
                pwm1 = 1500
                pwm3 = 1560
                if(cube == False and cube_last == True):
                    print(cube, cube_last,cube_name,cube_name_last)
                    state_now = 'cube_depth'
                    if(cube_name_last == 'triangle'):
                        red_blink()

                    if(cube_name_last == 'circular'):
                        green_blink()

            else:
                pwm2 = int(yaw_pid_partrol_pipline_light_fast.calculate(0.5,tube_center_last,1495))
                pwm3 = 1560

            time_over = time.time()
            # if(time_over - time_start >= 0.6):
            #     open_claw()
            #     time_begin = 0

        if state_now == 'cube_depth':
            if q_width.empty() == False:
                mean = q_width.get()
                print(mean)
                print(1)
                pwm3 = 1465
                pwm2= int(yaw_pid_partrol_pipline_light_slow.calculate(0.5,center,1500))
                print(2)
                if mean < 40:
                    pwm1= int(ud_pid.calculate(40,mean))
                    print(3)
                else:
                    state_now = 'cube_claw'
                    print(4)




        if state_now == 'cube_claw':    
            time_start = time.time()
            close_claw()
            state_now = 'cube_float'
        

        if state_now == 'cube_float':
            print(5)
            if q_width.empty() == False:
                mean = q_width.get()
                print(mean)
                pwm3 = 1500
                pwm2= int(yaw_pid_partrol_pipline_light_slow.calculate(0.5,center,1500))
                if mean > 35:
                    pwm1= int(ud_pid.calculate(35,mean))
                else:
                    state_now = 'pipline_light'
                print(1111111111111111111111)
            

        #aim(pwm1,pwm2,pwm3) #分别是上浮下潜、左右偏航、前进后退

'''
        if state_now == 'Turn_one':# turn
            print("Turn_one")
            turn()
            state_now = 'pipline_depth'
            angle = False
            angle_last = False


        if state_now == 'pipline_depth':
            if tube_center is not None: #patrol
                mean = q_width.get()
                center = (tube_center[0])/224
                tube_center_last = center
                pwm1= int(ud_pid.calculate(25,mean))
                print(mean)
                print(pwm1)
                pwm2 = int(yaw_pid_partrol_pipline_depth.calculate(0.5,center,1499))
                pwm3 = 1600
                if(cube == False and cube_last == True):
                    print(cube, cube_last,cube_name,cube_name_last)
                    if(cube_name_last == 'cube'):
                        red_on()
                        time.sleep(0.01)
                        red_off()
                        close_claw()
                        print("CLOSE!")
                        time_start = time.time()
                        #open_claw()
                    if(cube_name_last == 'circle'):
                        green_on()
                        time.sleep(0.01)
                        green_off()
                        close_claw()
                        print("CLOSE!")
                        time_start = time.time()
                        #open_claw()
            else:
                pwm2 = int(yaw_pid_partrol_pipline_depth.calculate(0.5,tube_center_last,1499))
                pwm3 = 1560
            time_over = time.time()
            if(time_over - time_start >= 0.6):
                open_claw()
                time_begin = 0
'''
        #aim(pwm1,pwm2,pwm3) #分别是上浮下潜、左右偏航、前进后退
