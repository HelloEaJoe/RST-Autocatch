import queue
import time
import threading
from pymavlink import mavutil
from visual.pipeline import *
from move.arm_disarm import arm, disarm
from move.move import *
from move.init import *
from pid_catalog_first.sheer_pid import *
from YOLOv5Litemaster.detectpipeline import *
from parameter.read_depth import *
from parameter.real_read_yaw import *


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


# 定义一个为了避免time.sleep()的定时运动的函数
def run_pwm_function(duration, pwm1, pwm2, pwm3):
    start_time_1 = time.time()
    end_time_1 = start_time_1 + 0.1
    while True:
        if end_time_1 - start_time_1 < duration:
            aim(pwm1, pwm2, pwm3)
            end_time_1 = time.time()
        else:
            break


if __name__ == '__main__':
    make_print_to_file(path='./')
    print("GO!")
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
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

    state_now = 'grip'
    current_state_index = 0  # 当前状态的索引
    print("game_begin")

    yaw_before_door = read_yaw_continually()
    while True:

        if state_now == 'grip':

            durtion = 5

            # if pink_ball_x >= 0.48 and pink_ball_x <= 0.52 and pink_ball_y >= 0.25 and pink_ball_y <= 0.35:
            #
            #     if rounds_for_small_ball == 0:
            #
            #         start_time = time.time()
            #         rounds_for_small_ball = 1
            #
            #     elif rounds_for_small_ball == 1:
            #
            #         if time.time() - start_time >= duration:
            #             grip = True
            # else:
            #
            #     rounds_for_small_ball = 0
            #     grip = False

            # 准备抓球
            if pink_small_ball is not True:
                aim(int(depth_pid.calculate(31, depth1, 1495)), 1500, 1550)

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
                    int(z_pid.calculate(0.1, pink_ball_y, 1495)))

                if grip and steady:

                    grip()
                    run_pwm_function(1, int(depth_pid.calculate(35, depth1, 1495)), 1550, 1500)
                    yaw_to_pitch = read_yaw_continually(True)

                    break































































































