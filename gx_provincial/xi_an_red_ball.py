import queue
import time
import threading
from pymavlink import mavutil
from visual.pipeline import*
from move.arm_disarm import arm,disarm
from move.move import *
from move.init import *
from pid_catalog_first.sheer_pid import *
# from YOLOv5Litemaster.detectpipeline import *
from parameter.read_depth import *
from parameter.real_read_yaw import *
from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *

#电脑的模型路径
# model_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\weights\best_1.onnx"
# label_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\data\xian.yaml"
#pi的模型路径
model_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/weights/best_1.onnx"
label_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/data/xian.yaml"


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
def run_pwm_function(duration, pwm1, pwm2, pwm3, pwm4):
    start_time_1 = time.time()
    end_time_1 = start_time_1 + 0.1
    while True:
        depth1 = read_depth_continual(True)
        robot_yaw = read_yaw_continually(True)
        if end_time_1  - start_time_1< duration:
            aim(pwm1, pwm2, pwm3)
            Heng(pwm4)
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
    rounds_for_door = 0
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
    state_now = 'HIT'



    net = yolov5_lite(model_path, label_path, confThreshold=0.85, nmsThreshold=0.5)

    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()

    print("game_begin")


    first_yaw = read_yaw_continually()
    while True:

        if state_now == 'HIT':
            print(f"state is {state_now} and i'll search the ball and hit")

            process_frame = net.detect(frame)
            rectangle_coordinate = net.get_rectangle_coordinate()
            owning_classes = net.get_owning_classes()
            center = net.get_center()
            area = net.get_area()

            for i in range(len(rectangle_coordinate)):
                if owning_classes[i] == 'redball':
                    red_big_ball = True
                    red_ball_x, red_ball_y, ball_width, ball_height = rectangle_coordinate[i]
                    center_x, center_y = center[i]
                    red_ball_area = area[i]
                    break

            if red_big_ball is not True:
                robot_yaw = read_yaw_continually(True)
                depth1 = read_depth_continual(True)
                pwm1 = int(depth_pid.calculate(60, depth1, 1495))

                aim(pwm1, 1500, 1550)

            if red_big_ball is True:

                if red_ball_area <= 0.8:
                    # red_ball_x = 0.5
                    # red_ball_y = 0.5
                    depth1 = read_depth_continual(True)
                    pwm1 = int(depth_pid.calculate(60, depth1, 1495))
                    pwm2 = int(yaw_pid.calculate(0.5, center_x, 1495))
                    aim(pwm1, pwm2, 1550)
                    last_robot_yaw = read_yaw_continually(True)

                else:

                    #撞球动作
                    run_pwm_function(2, int(depth_pid.calculate(60, depth1, 1495)),
                                     1500, 1550, 1500)
                    #转换状态到patroll_lines_to_next

                    print(f"state changes to patroll_lines_to_next")
                    break

