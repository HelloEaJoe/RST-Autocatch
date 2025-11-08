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

    state_now = 'through_low_door'
    current_state_index = 0  # 当前状态的索引
    print("game_begin")


    yaw_before_door = read_yaw_continually()
    while True:

        if state_now == 'through_low_door':

            if door_previous is not True:
                print("没门")
                robot_yaw = read_yaw_continually(True)
                pwm1 = int(depth_pid.calculate(65, depth1, 1495))
                pwm2 = int(yaw_pid.calculate(yaw_before_door, robot_yaw, 1495))
                aim(pwm1, pwm2, 1550)

            if door_previous == True:


                if door_now == True:
                    print("有门")
                    if rounds_for_door == 0:
                        pwm1 = int(depth_pid.calculate(65, depth1, 1495))
                        pwm2 = int(yaw_pid.calculate(0.5, aim_door_x, 1495))
                        pwm4 = int(heng_pid.calculate(0.5, aim_door_x, 1495))

                        run_pwm_function(2, pwm1, pwm2, 1500, pwm4)
                        rounds_for_door = 1

                    if rounds_for_door == 1:
                        pwm1 = int(depth_pid.calculate(65, depth1, 1495))
                        pwm2 = int(yaw_pid.calculate(0.5, aim_door_x, 1495))
                        pwm4 = int(heng_pid.calculate(0.5, aim_door_x, 1495))

                        run_pwm_function(1, pwm1, pwm2, 1600, pwm4)
                        run_pwm_function(1, pwm1, pwm2, 1400, pwm4)

                        rounds_for_door = 2

                    if rounds_for_door == 2:
                        while True:

                            if door_area <= 0.8:

                                pwm1 = int(depth_pid.calculate(65, depth1, 1495))
                                pwm2 = int(yaw_pid.calculate(0.5, aim_door_x, 1495))
                                pwm4 = int(heng_pid.calculate(0.5, aim_door_x, 1495))
                                aim(pwm1, pwm2, 1600)
                                Heng(pwm4)
                                yaw_before_through_door = read_yaw_continually(True)

                            else:
                                pwm1 = int(depth_pid.calculate(65, depth1, 1495))
                                pwm2 = int(yaw_pid.calculate(yaw_before_through_door, robot_yaw, 1495))

                                run_pwm_function(3, pwm1, pwm2, 1600, 1500)
                                rounds_for_door == 0

                                break

































































































