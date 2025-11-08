import pygame
import sys
from pid_catalog_first.sheer_pid import *
from pymavlink import mavutil
from parameter.read_depth import *
import time
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


# 解锁
def arm():
    # Create the connection
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    # master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

    # Arm
    # master.arducopter_arm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    # master.motors_armed_wait()
    print('Armed!')


# 上锁
def disarm():
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    master.motors_disarmed_wait()


# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
# 初始pwm设置为1500
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """

    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(9)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        *rc_channel_values)  # RC channel list, in microseconds.


def static():
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(3, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)


def aim(pwm1, pwm2, pwm3, pwm4):
    # 上浮下潜
    set_rc_channel_pwm(3, pwm1)  # 上浮下潜
    set_rc_channel_pwm(4, pwm2)  # 左拐右拐
    set_rc_channel_pwm(5, pwm3)  # 前进后退
    set_rc_channel_pwm(6, pwm4)  # 左移右移
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)


#     print(pwm1)

# set_rc_channel_pwm(9, 1100)
# set_rc_channel_pwm(10, 1100)

# def lights_1_level(pwm):
#     set_rc_channel_pwm(9, pwm)

# def lights_2_level(pwm):
#     set_rc_channel_pwm(10, pwm)


def init():
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(3, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)
    set_rc_channel_pwm(7, 1500)
    set_rc_channel_pwm(8, 1500)
    set_rc_channel_pwm(9, 1100)
    # set_rc_channel_pwm(10,1100)
    # set_rc_channel_pwm(11,1500)
    print('init')
    return True


def init_force():
    while True:
        set_rc_channel_pwm(1, 1500)
        set_rc_channel_pwm(2, 1500)
        set_rc_channel_pwm(3, 1500)
        set_rc_channel_pwm(4, 1500)
        set_rc_channel_pwm(5, 1500)
        set_rc_channel_pwm(6, 1500)
        set_rc_channel_pwm(7, 1500)
        set_rc_channel_pwm(8, 1500)
        set_rc_channel_pwm(9, 1100)
        # set_rc_channel_pwm(10,1100)
        # set_rc_channel_pwm(11,1500)
        print('init_force')


pygame.init()
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption("pygame处理事件")
a = 1500  # 固定值
b = 1400
c = 1600
forward = Falsec = 1600
back = Falsec = 1600
left = Falsec = 1600
right = False
turn_left = False
turn_right = False
close = False
open= False

if __name__ == '__main__':
    print("GO!")
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    disarm()
    init()
    arm()
    while True:
        for event in pygame.event.get():
            start = time.time()
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == 119:
                    print("forward")
                    depth1 = read_depth_continual(True)
                    aim(int(depth_pid.calculate(21, depth1, 1495)), a, c, a)
                if event.key == 97:
                    print("back")
                    depth1 = read_depth_continual(True)
                    aim(int(depth_pid.calculate(21, depth1, 1495)), a, b, a)
                if event.key == 115:
                    print("left")
                    depth1 = read_depth_continual(True)
                    aim(int(depth_pid.calculate(21, depth1, 1495)), a, a, b)
                if event.key == 100:
                    print("right")
                    depth1 = read_depth_continual(True)
                    aim(int(depth_pid.calculate(21, depth1, 1495)), a, a, c)
                if event.key == 105:
                    print("come_up")
                    aim(int(depth_pid.calculate(21, depth1, 1495)), a, a, a)
                if event.key == 106:
                    print("come_dowm")
                    aim(b, a, a, a)
                if event.key == 107:
                    print("turn_left")
                    depth1 = read_depth_continual(True)
                    aim(int(depth_pid.calculate(21, depth1, 1495)), b, a, a)
                if event.key == 108:
                    print("turn_right")
                    depth1 = read_depth_continual(True)
                    aim(int(depth_pid.calculate(21, depth1, 1495)), c, a, a)

            elif event.type == pygame.KEYUP:
                if event.key == 119:
                    aim(1500,1500,1500,1500)
                if event.key == 97:
                    aim(1500,1500,1500,1500)
                if event.key == 115:
                    aim(1500,1500,1500,1500)
                if event.key == 100:
                    aim(1500,1500,1500,1500)
                if event.key == 105:
                    aim(1500,1500,1500,1500)
                if event.key == 106:
                    aim(1500,1500,1500,1500)
                if event.key == 107:
                    aim(1500,1500,1500,1500)
                if event.key == 108:
                    aim(1500,1500,1500,1500)
            pygame.display.update()

