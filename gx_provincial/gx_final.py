import queue
import time
import threading
from pymavlink import mavutil
from visual.pipeline import*
#from YOLOv5Litemaster.pipeline_jiao import*
from move.arm_disarm import arm,disarm
from move.move import *
from move.init import *
from pid_catalog_first.set_deepth import*
from pid_catalog_first.sheer_pid import *
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
    substate_now = 'patrol' #patrol，cube_depth，cube_claw，cube_float
    tube_center_last = 0 
    global pwm1,pwm2,pwm3,flag
    pwm1,pwm2,pwm3,flag=1500,1500,1500,0
    net = yolov5_lite("best-4.21.onnx", "cube.names", 0.25, 0.3)
    camera = cv2.VideoCapture(0)

    claw_time_start = 0
    claw_time_over = 0
    cube_time_start = 0
    cube_time_over = 0

    flag_cube = 0

    count = 0
    m = means = 0
    cube = False
    cube_last = False
    angle = False
    angle_last = False
    cube_name = None
    cube_name_last = None
    angle_point = None
    angle_lastpoint = None
    aim(1500,1500,1500)
    while True:
        print(state_now)
        print(substate_now)
        ret, frame = camera.read()
        angle_last = angle
        cube_last = cube
        cube_name_last = cube_name
        if ret:
            frame = cv2.resize(frame, (320, 320))
            frame2 = frame.copy()
            count = (count + 1) % 3
            if count == 0:
                frame2, angle, angle_point, cube, cube_name = net.detect(frame2, 150, 50)
                #print(angle, angle_point, cube, cube_name)
                angle_lastpoint = angle_point
            else:
                cube_name = cube_name_last
                angle_point = angle_lastpoint
            m, means = patrol(frame, True)
        #cv2.imshow("img2", frame2)
        #cv2.waitKey(1)

        claw_time_over = time.time()
        if (claw_time_over-claw_time_start) > 0.8:
            open_claw()

        tube_center = q_center.get()
        if state_now == 'pipline_light':
            if substate_now == 'patrol':
                if tube_center is not None: #patrol
                    center = (tube_center[0])/320
                    tube_center_last = center
                    print(center)
                    if angle == False:
                        pwm2 = int(yaw_pid_partrol_pipline_light_fast.calculate(0.5,center,1495))
                    else:
                        pwm2 = int(yaw_pid_partrol_pipline_light_slow.calculate(0.5,center,1495))
                    if cube == True or angle == True:
                        pwm3 = 1560
                    else:
                        pwm3 = 1570
                    if(cube == False and cube_last == True):
                        print(cube, cube_last,cube_name,cube_name_last)
                        substate_now = 'cube_depth'
                        if(cube_name_last == 'cube'):
                            green_blink()
                        if(cube_name_last == 'circle' or cube_name_last == 'circular'): 
                            red_blink()
                        if(cube_name_last == 'triangle'):
                            blue_blink()
                else:
                    pwm2 = int(yaw_pid_partrol_pipline_light_fast.calculate(0.5,tube_center_last,1495))
                    pwm3 = 1550
            
            if flag_cube == 0 :
                cube_time_start = time.time()
                print('Start!')

            if substate_now == 'cube_depth':
                flag_cube = 1
                if q_width.empty() == False:
                    if tube_center is not None: #patrol
                        center = (tube_center[0])/320
                        mean = q_width.get()
                        print(mean)
                        pwm3 = 1460 #!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        pwm2= int(yaw_pid_partrol_pipline_light_slow.calculate(0.5,center,1500))
                        if mean < 58:
                            pwm1= int(ud_pid.calculate(63,mean))       #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        else:
                            substate_now = 'cube_claw'
            
            if substate_now == 'cube_claw':
                claw_time_start = time.time()
                close_claw()
                time.sleep(1)
                open_claw()
                time.sleep(1)
                close_claw()
                substate_now = 'cube_float'

            if substate_now == 'cube_float':
                if q_width.empty() == False:
                    if tube_center is not None: #patrol
                        center = (tube_center[0])/320
                        mean = q_width.get()
                        print(mean)
                        pwm3 = 1500
                        pwm2= int(yaw_pid_partrol_pipline_light_slow.calculate(0.5,center,1500))
                        if mean > 46:
                            pwm1= int(ud_pid.calculate(48,mean))   #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        else:
                            substate_now = 'patrol'
                            pwm1 = 1500

            cube_time_over = time.time()
            if cube_time_over - cube_time_start > 3.5: #!!!!!!!!!!!!!!!!!!!!!!!!!
                print(cube_time_over - cube_time_start)
                flag_cube = 0
                print('Over!')
                substate_now = 'patrol'
                pwm1 = 1500

            if(angle == False and angle_last ==True):
                    state_now = 'Turn_one'
                    print("Turn_one")

            aim(pwm1,pwm2,pwm3) #分别是上浮下潜、左右偏航、前进后退
        
        if state_now == 'Turn_one':# turn
            print("Turn_one")
            turn_first()
            state_now = 'pipline_middle'
            substate_now = 'patrol'
            angle = False
            angle_last = False


        if state_now == 'pipline_middle':
            if tube_center is not None: #patrol
                if q_width.empty() == False:
                    mean = q_width.get()
                    center = (tube_center[0])/320
                    tube_center_last = center
                    pwm1= int(ud_pid.calculate(34,mean))
                    print(center)
                    if angle == False:
                        pwm2 = int(yaw_pid_partrol_pipline_light_fast.calculate(0.5,center,1495))
                    else:
                        pwm2 = int(yaw_pid_partrol_pipline_light_slow.calculate(0.5,center,1495))
                    print(pwm2)
                    if cube == True or angle == True:
                        pwm3 = 1550
                    else:
                        pwm3 = 1570
                if(cube == False and cube_last == True):
                    print(cube, cube_last,cube_name,cube_name_last)
                    if(cube_name_last == 'cube'):
                        green_blink()
                    if(cube_name_last == 'circle' or cube_name_last == 'circular'): 
                        red_blink()
                    if(cube_name_last == 'triangle'):
                        blue_blink()
            else:
                pwm2 = int(yaw_pid_partrol_pipline_light_fast.calculate(0.5,tube_center_last,1495))
                pwm3 = 1540
            if(angle == False and angle_last ==True):
                    state_now = 'Turn_two'


            if(angle == False and angle_last ==True):
                    state_now = 'Turn_two'
 
            aim(pwm1,pwm2,pwm3) #分别是上浮下潜、左右偏航、前进后退        

        

        if state_now == 'Turn_two':# turn
            print("Turn_two")
            turn_depth_first()
            state_now = 'pipline_depth'
            substate_now = 'patrol'
            angle = False
            angle_last = False     

        if state_now == 'pipline_depth':
            if tube_center is not None: #patrol
                if q_width.empty() == False:
                        mean = q_width.get()
                        center = (tube_center[0])/320
                        tube_center_last = center
                        pwm1= int(ud_pid.calculate(40,mean))
                        print(mean)
                        print(pwm1)
                        pwm2 = int(yaw_pid_partrol_pipline_depth.calculate(0.5,center,1499))
                        pwm3 = 1575
                if(cube == False and cube_last == True):
                    print(cube, cube_last,cube_name,cube_name_last)
                    if(cube_name_last == 'cube'):
                        green_blink()
                    if(cube_name_last == 'circle' or cube_name_last == 'circular'): 
                        red_blink()
                    if(cube_name_last == 'triangle'):
                        blue_blink()
            else:
                pwm2 = int(yaw_pid_partrol_pipline_depth.calculate(0.5,tube_center_last,1499))
                pwm3 = 1550

            aim(pwm1,pwm2,pwm3) #分别是上浮下潜、左右偏航、前进后退
            
            
            
       
