import queue
import time
import threading
from pymavlink import mavutil
from visual.pipeline import*
from move.arm_disarm import arm,disarm
from move.move import *
from move.init import *
from pid_catalog_first.sheer_pid import *
from YOLOv5Litemaster.detectpipeline import *
from parameter.read_depth import *
from parameter.real_read_yaw import *




#定义碰撞的蓝色球和红色球
class BigBall:
    def __init__(self, red_ball_x, red_ball_y, red_ball_area, blue_ball_x, blue_ball_y, blue_ball_area):
        # Initialize the properties of the red ball
        self.red_ball = {
            'x': red_ball_x,
            'y': red_ball_y,
            'area': red_ball_area
        }
        # Initialize the properties of the blue ball
        self.blue_ball = {
            'x': blue_ball_x,
            'y': blue_ball_y,
            'area': blue_ball_area
        }

    def display_info(self):
        # Print the information of the red and blue balls
        self.print_red_ball_info()
        self.print_blue_ball_info()

    def print_red_ball_info(self):
        # Print the information of the red ball
        print(f"Red Ball coordinates: ({self.red_ball['x']}, {self.red_ball['y']}), Area: {self.red_ball['area']}")

    def print_blue_ball_info(self):
        # Print the information of the blue ball
        print(f"Blue Ball coordinates: ({self.blue_ball['x']}, {self.blue_ball['y']}), Area: {self.blue_ball['area']}")



# Assuming the following coordinates and areas are obtained by the YoloV5 algorithm
red_ball_x, red_ball_y, red_ball_area = 10, 20, 100
blue_ball_x, blue_ball_y, blue_ball_area = 30, 40, 150

# Create an instance of BigBall
big_balls = BigBall(red_ball_x, red_ball_y, red_ball_area, blue_ball_x, blue_ball_y, blue_ball_area)

#展示一些这个类的用法，方便后续调用
# # Display the information of the big balls
# big_balls.display_info()
#
# # Alternatively, you can call the individual methods to print the information separately
# big_balls.print_red_ball_info()
# big_balls.print_blue_ball_info()



# class SmallBall:
#     def __init__(self, pink_ball_x, pink_ball_y, pink_ball_area, yellow_ball_x, yellow_ball_y, yellow_ball_area):
#         # Initialize the properties of the pink ball
#         self.pink_ball = {
#             'x': pink_ball_x,
#             'y': pink_ball_y,
#             'area': pink_ball_area
#         }
#         # Initialize the properties of the yellow ball
#         self.yellow_ball = {
#             'x': yellow_ball_x,
#             'y': yellow_ball_y,
#             'area': yellow_ball_area
#         }
#
#     def display_info(self):
#         # Print the information of the pink and yellow balls
#         self.print_pink_ball_info()
#         self.print_yellow_ball_info()
#
#     def print_pink_ball_info(self):
#         # Print the information of the pink ball
#         print(f"Pink Ball coordinates: ({self.pink_ball['x']}, {self.pink_ball['y']}), Area: {self.pink_ball['area']}")
#
#     def print_yellow_ball_info(self):
#         # Print the information of the yellow ball
#         print(f"Yellow Ball coordinates: ({self.yellow_ball['x']}, {self.yellow_ball['y']}), Area: {self.yellow_ball['area']}")
#
# # Assuming the following coordinates and areas are obtained by the YoloV5 algorithm
# pink_ball_x, pink_ball_y, pink_ball_area = 10, 20, 100
# yellow_ball_x, yellow_ball_y, yellow_ball_area = 30, 40, 150
#
# # Create an instance of SmallBall
# small_balls = SmallBall(pink_ball_x, pink_ball_y, pink_ball_area, yellow_ball_x, yellow_ball_y, yellow_ball_area)
#
# # Display the information of the small balls
# small_balls.display_info()
#
# # Alternatively, you can call the individual methods to print the information separately
# small_balls.print_pink_ball_info()
# small_balls.print_yellow_ball_info()


class SmallBall:
    def __init__(self, pink_ball_x, pink_ball_y, yellow_ball_x, yellow_ball_y):
        # Initialize the properties of the pink ball
        self.pink_ball = {
            'x': pink_ball_x,
            'y': pink_ball_y
        }
        # Initialize the properties of the yellow ball
        self.yellow_ball = {
            'x': yellow_ball_x,
            'y': yellow_ball_y
        }

    def display_info(self):
        # Print the information of the pink and yellow balls
        self.print_pink_ball_info()
        self.print_yellow_ball_info()

    def print_pink_ball_info(self):
        # Print the information of the pink ball
        print(f"Pink Ball coordinates: ({self.pink_ball['x']}, {self.pink_ball['y']})")

    def print_yellow_ball_info(self):
        # Print the information of the yellow ball
        print(f"Yellow Ball coordinates: ({self.yellow_ball['x']}, {self.yellow_ball['y']})")

# Assuming the following coordinates are obtained by the YoloV5 algorithm
pink_ball_x, pink_ball_y = 10, 20
yellow_ball_x, yellow_ball_y = 30, 40

#举例几种用法
# # Create an instance of SmallBall
# small_balls = SmallBall(pink_ball_x, pink_ball_y, yellow_ball_x, yellow_ball_y)
#
# # Display the information of the small balls
# small_balls.display_info()
#
# # Alternatively, you can call the individual methods to print the information separately
# small_balls.print_pink_ball_info()
# small_balls.print_yellow_ball_info()
#

def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds,  # PWM pulse-width
        0, 0, 0, 0, 0  # unused parameters
    )



# 定义函数，按顺序切换到下一个状态并打印
def switch_to_next_state():
    global current_state_index,state_now
    current_state_index = (current_state_index + 1) % len(STATES)
    print(f"State changes to {STATES[current_state_index]}")
    state_now = STATES[current_state_index]


#定义一个为了避免time.sleep()的定时运动的函数
def run_pwm_function(duration, pwm1, pwm2, pwm3):
    start_time_1 = time.time()
    end_time_1 = start_time_1 + 0.1
    while True:
        if end_time_1  - start_time_1< duration:
            aim(pwm1, pwm2, pwm3)
            end_time_1 = time.time()
        else:
            break

def grip():
    set_servo_pwm(2, 1000)
    time.sleep(0.5)
    aim(int(depth_pid.calculate(21, depth1, 1495)),
        int(yaw_pid.calculate(0.5, pink_ball_x, 1495)),
        int(z_pid.calculate(0.3, pink_ball_y, 1495)))


def pitch():
    set_servo_pwm(2, 1500)
    time.sleep(0.5)
    aim(int(depth_pid.calculate(21, depth1, 1495)),
        int(yaw_pid.calculate(0.5, pink_ball_x, 1495)),
        int(z_pid.calculate(0.3, pink_ball_y, 1495)))




if __name__=='__main__':
    make_print_to_file(path='./')
    print("GO!")
    master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
    master.wait_heartbeat()
    disarm()
    init()
    arm()

    time_start_for_lines = 0
    time_for_lines = 0
    rounds_for_lines = 0
    rounds_for_big_ball = 0
    rounds_for_middle_lines = 0
    rounds_for_small_ball = 0
    patrol_line_center = 0
    rounds_for_steady = 0
    aim_door_x = 0
    aim_door_y = 0
    door_area = 0
    duration = 0
    direction_to_start = 0
    Collection_bin_x = 0
    Collection_bin_y = 0
    Collection_bin_area = 0
    depth_to_collection = 0


    global yaw_before_door
    global yaw_before_through_door
    global yaw_to_pitch
    global depth_before_through_door
    global depth1

    pink_small_ball = True
    red_big_ball = True
    blue_big_ball = True
    yellow_small_ball = True
    pink_small_ball = True
    line_now = True
    line_previous = True
    door_now = True
    door_previous = True
    grip = True
    steady = True
    Collection_bin = True




    #状态设置
    STATES = ['patroll_lines_to_hit',
              'HIT',
              'patroll_lines_to_next',
              'through_high_door',
              'patroll_lines_to_next',
              'through_low_door',
              'patroll_lines_to_next',
              'through_high_door',
              'patroll_lines_to_next',
              'through_low_door',
              'patroll_lines_to_next',
              'grip',
              'pitch',
              'back_to_begin',
              ]
    current_state_index = 0  # 当前状态的索引
    print("game_begin")


    first_yaw = read_yaw_continually()
    while True:

        if state_now == 'patroll_lines_to_hit':

            print(f"state is {state_now} and i'll dive three seconds")
            run_pwm_function(3, 1300, 1500, 1600)

            if line_now and line_previous is not True:
                depth1 = read_depth_continual(True)
                global robot_yaw
                robot_yaw = read_yaw_continually()

                print("find the first line")
                pwm1 = int(depth_pid.calculate(10, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(first_yaw, robot_yaw, 1495))
                aim(pwm1, pwm2, 1550)





            if line_previous and line_now is True :

                if rounds_for_lines == 0 :

                    time_start_for_lines = time.time()
                    time_for_lines = time.time()
                    rounds_for_lines += 1

                else :

                    time_for_lines = time.time()
                    rounds_for_lines += 1

                global duration_time
                duration_time = time_for_lines - time_start_for_lines
                # 保持通过深度计稳定深度并且通过视觉保持稳定航向

                depth1 = read_depth_continual(True)
                pwm1 = int(depth_pid.calculate(10, depth1, 1495))
                pwm2 = int(patrol_line.calculate(0.5, patrol_line_center, 1495))

                aim(pwm1, pwm2, 1600)
                global last_yaw
                last_yaw = read_yaw_continually()

            if line_previous is True and line_now is False and duration_time >=4:

                depth1 = read_depth_continual(True)
                pwm1 = int(depth_pid.calculate(60, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(last_yaw, robot_yaw, 1495))
                aim(pwm1, pwm2, 1550)
                rounds_for_lines = 0
                switch_to_next_state()  #转换到撞球状态





        if state_now == 'HIT':
            print(f"state is {state_now} and i'll search the ball and hit")
            run_pwm_function(0.2, int(depth_pid.calculate(60, depth1, 1495)),1550, 1550)

            if red_big_ball is not True:


                depth1 = read_depth_continual(True)
                pwm1 = int(depth_pid.calculate(60, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(last_yaw, robot_yaw, 1495))
                aim(pwm1, pwm2, 1550)

            if red_big_ball is True:

                red_ball_x = 0.5
                red_ball_y = 0.5
                # 确定球中心深度
                if rounds_for_big_ball == 0:

                    while True:

                        if red_ball_y >= 0.508:

                            aim(1450, 1500, 1500)
                        if red_ball_y <= 0.492:

                            aim(1550, 1500, 1500)

                        if red_ball_y <= 0.508 and red_ball_y >= 0.492:

                            global big_ball_depth
                            big_ball_depth = read_depth_continual(True)
                            depth1 = read_depth_continual(True)
                            pwm1 = int(depth_pid.calculate(big_ball_depth, depth1, 1495))
                            pwm2 = int(yaw_pid.calculate(0.5, red_ball_x , 1495))
                            aim(pwm1, pwm2, 1550)
                            break

                    rounds_for_big_ball += 1
                else:

                    if red_ball_area <= 0.8:

                        depth1 = read_depth_continual(True)
                        pwm1 = int(depth_pid.calculate(big_ball_depth, depth1, 1495))
                        pwm2 = int(yaw_pid.calculate(0.5, red_ball_x, 1495))
                        aim(pwm1, pwm2, 1550)
                        last_robot_yaw = read_yaw_continually()


                    else:

                        #撞球动作
                        run_pwm_function(2, int(depth_pid.calculate(big_ball_depth, depth1, 1495)),
                                         int(yaw_pid.calculate(robot_yaw, last_robot_yaw, 1495)), 1550)
                        #转换状态到patroll_lines_to_next
                        switch_to_next_state()
                        print(f"state changes to patroll_lines_to_next")





        if state_now == 'patroll_lines_to_next':

            if rounds_for_middle_lines == 0:

                run_pwm_function(2, int(depth_pid.calculate(10, depth1, 1495)), 1550, 1500)
                rounds_for_middle_lines += 1

            elif rounds_for_middle_lines == 2:

                run_pwm_function(2, int(depth_pid.calculate(10, depth1, 1495)), 1550, 1500)
                rounds_for_middle_lines += 1

            elif rounds_for_middle_lines == 4:

                run_pwm_function(2, int(depth_pid.calculate(10, depth1, 1495)), 1550, 1500)
                rounds_for_middle_lines += 1

            elif rounds_for_middle_lines == 6:

                run_pwm_function(2, int(depth_pid.calculate(10, depth1, 1495)), 1550, 1500)
                rounds_for_middle_lines += 1

            elif rounds_for_middle_lines == 8:

                run_pwm_function(2, int(depth_pid.calculate(10, depth1, 1495)), 1550, 1500)
                rounds_for_middle_lines += 1



            if line_now and line_previous is not True:
                # global robot_yaw
                robot_yaw = read_yaw_continually()

                print("find the line")
                pwm1 = int(depth_pid.calculate(10, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(first_yaw, robot_yaw, 1495))
                aim(pwm1, pwm2, 1550)

            if line_previous and line_now is True:

                if rounds_for_lines == 0:

                    time_start_for_lines = time.time()
                    time_for_lines = time.time()
                    rounds_for_lines += 1

                else:

                    time_for_lines = time.time()
                    rounds_for_lines += 1

                global duration_time
                duration_time = time_for_lines - time_start_for_lines
                # 保持通过深度计稳定深度并且通过视觉保持稳定航向
                global depth1
                depth1 = read_depth_continual(True)
                pwm1 = int(depth_pid.calculate(10, depth1, 1495))
                pwm2 = int(patrol_line.calculate(0.5, patrol_line_center, 1495))

                aim(pwm1, pwm2, 1600)
                global last_yaw
                last_yaw = read_yaw_continually()

            if line_previous is True and line_now is False and duration_time >= 4:

                depth1 = read_depth_continual(True)
                pwm1 = int(depth_pid.calculate(60, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(last_yaw, robot_yaw, 1495))
                aim(pwm1, pwm2, 1550)

                if rounds_for_middle_lines == 1:

                    run_pwm_function(2, int(depth_pid.calculate(45, depth1, 1495)), 1550, 1500)
                    rounds_for_middle_lines += 1
                    yaw_before_door = read_yaw_continually()

                elif rounds_for_middle_lines == 3:

                    run_pwm_function(2, int(depth_pid.calculate(65, depth1, 1495)), 1550, 1500)
                    rounds_for_middle_lines += 1
                    yaw_before_door = read_yaw_continually()

                elif rounds_for_middle_lines == 5:

                    run_pwm_function(2, int(depth_pid.calculate(45, depth1, 1495)), 1550, 1500)
                    rounds_for_middle_lines += 1
                    yaw_before_door = read_yaw_continually()

                elif rounds_for_middle_lines == 7:

                    run_pwm_function(2, int(depth_pid.calculate(65, depth1, 1495)), 1550, 1500)
                    rounds_for_middle_lines += 1
                    yaw_before_door = read_yaw_continually()

                elif rounds_for_middle_lines == 9:

                    run_pwm_function(2, int(depth_pid.calculate(10, depth1, 1495)), 1550, 1500)


                switch_to_next_state()



        if state_now == 'through_high_door':

            if door_previous is False and door_now is False:

                pwm1 = int(depth_pid.calculate(65, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(yaw_before_door, robot_yaw, 1495))
                aim(pwm1, pwm2 ,1550)

            if door_now is True and door_area <= 0.8:

                pwm1 = int(depth_pid.calculate(0.5, aim_door_y, 1495))
                pwm2 = int(yaw_pid.calculate(0.5, aim_door_x, 1495))
                aim(pwm1, pwm2, 1550)
                yaw_before_through_door = read_yaw_continually()
                depth_before_through_door = read_depth_continual(True)


            if door_now is True and door_area >= 0.8:

                print("要过门啦")
                robot_yaw = read_yaw_continually()
                depth1 = read_depth_continual(True)
                #过门动作
                run_pwm_function(3, int(depth_pid.calculate(depth_before_through_door, depth1, 1495)),
                                 int(yaw_pid.calculate(yaw_before_through_door, robot_yaw, 1495)), 1600)

                switch_to_next_state()


        if state_now == 'through_low_door':

            if door_previous is False and door_now is False:

                pwm1 = int(depth_pid.calculate(45, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(yaw_before_door, robot_yaw, 1495))
                aim(pwm1, pwm2, 1550)

            if door_now is True and door_area <= 0.8:

                pwm1 = int(depth_pid.calculate(0.5, aim_door_y, 1495))
                pwm2 = int(yaw_pid.calculate(0.5, aim_door_x, 1495))
                aim(pwm1, pwm2, 1550)
                yaw_before_through_door = read_yaw_continually()
                depth_before_through_door = read_depth_continual(True)


            if door_now is True and door_area >= 0.8:

                print("要过门啦")
                robot_yaw = read_yaw_continually()
                depth1 = read_depth_continual(True)
                #过门动作
                run_pwm_function(3, int(depth_pid.calculate(depth_before_through_door, depth1, 1495)),
                                 int(yaw_pid.calculate(yaw_before_through_door, robot_yaw, 1495)), 1600)

                switch_to_next_state()


        if state_now == 'grip':

            durtion = 5

            if pink_ball_x >= 0.48 and pink_ball_x <= 0.52 and pink_ball_y >= 0.25 and pink_ball_y <=0.35:

                if rounds_for_small_ball == 0:

                    start_time = time.time()
                    rounds_for_small_ball += 1

                elif rounds_for_small_ball >= 1:

                    if time.time() - start_time >= duration:

                        grip = True
            else:

                rounds_for_small_ball = 0
                grip = False




            #准备抓球
            if pink_small_ball is not True:

                aim(int(depth_pid.calculate(31, depth1, 1495)),1500 ,1550)

            if pink_small_ball is True:

                aim(int(depth_pid.calculate(31, depth1, 1495)),
                    int(yaw_pid.calculate(0.5, pink_ball_x, 1495)),
                    int(z_pid.calculate(0.3, pink_ball_y, 1495)))


                if pink_ball_x >= 0.48 and pink_ball_x <= 0.52 and pink_ball_y >= 0.25 and pink_ball_y <= 0.35:

                    if rounds_for_small_ball == 0:

                        start_time = time.time()
                        rounds_for_small_ball += 1

                    elif rounds_for_small_ball >= 1:

                        if time.time() - start_time >= duration:
                            grip = True
                else:

                    rounds_for_small_ball = 0
                    grip = False

                if depth1 >= 30 and depth1 <= 31:

                    if rounds_for_steady == 0:

                        start_time = time.time()
                        rounds_for_steady += 1

                    elif rounds_for_steady >= 1:

                        if time.time() - start_time >= duration:
                            steady = True
                else:

                    rounds_for_steady = 0
                    steady = False

                aim(int(depth_pid.calculate(31, depth1, 1495)),
                    int(yaw_pid.calculate(0.5, pink_ball_x, 1495)),
                    int(z_pid.calculate(0.3, pink_ball_y, 1495)))

                if grip and steady:

                    grip()
                    run_pwm_function(1, int(depth_pid.calculate(35, depth1, 1495)),1550, 1500)
                    yaw_to_pitch = read_yaw_continually()
                    switch_to_next_state()



        if state_now == 'pitch':

            if Collection_bin == False:

                pwm1 = int(depth_pid.calculate(35, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(yaw_to_pitch, robot_yaw, 1495))

                aim(pwm1, pwm2, 1500)

            else :
                if Collection_bin_area <= 0.8:

                    pwm1 = int(depth_pid.calculate(0.5, Collection_bin_y, 1495))
                    pwm2 = int(yaw_pid.calculate(0.5, Collection_bin_x, 1495))
                    aim(pwm1, pwm2, 1550)
                    yaw_to_pitch = read_yaw_continually()
                    depth_to_pitch = read_depth_continual(True)
                else :

                    depth1 = read_depth_continual(True)
                    pwm1 = int(depth_pid.calculate(depth_to_pitch, depth1, 1495))
                    pwm2 = int(yaw_pid.calculate(yaw_to_pitch, robot_yaw, 1495))
                    run_pwm_function(2, pwm1, pwm2, 1600)
                    pitch()
                    run_pwm_function(2, pwm1, pwm2, 1400)
                    switch_to_next_state()







        if state_now == 'back_to_begin':

            run_pwm_function(int(depth_pid.calculate(50, depth1, 1495)),
                                int(yaw_pid.calculate(direction_to_start, robot_yaw, 1495)),1600)




























































































