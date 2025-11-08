import pygame
import sys
from pymavlink import mavutil
import time


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
b = 1300
c = 1700

forward = False
back = False
left = False
right = False
turn_left = False
turn_right = False
come_up = False
come_dowm = False

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
                    # print("向前运动")
                    # aim(a, a, c, a)
                    forward = True
                if event.key == 97:
                    # print("向左运动")
                    # aim(a, a, a, b)
                    left = True
                if event.key == 115:
                    # print("向后运动")
                    # aim(a, a, b, a)
                    back = True
                if event.key == 100:
                    # print("向右运动")
                    # aim(a, a, a, c)
                    right = True
                if event.key == 105:
                    # print("上浮")
                    # aim(c, a, a, a)
                    come_up = True
                if event.key == 106:
                    # print("向左拐")
                    # aim(a, b, a, a)
                    turn_left = True
                if event.key == 107:
                    # print("下潜")
                    # aim(b, a, a, a)
                    come_dowm = True
                if event.key == 108:
                    # print("向右拐")
                    # aim(a, c, a, a)
                    turn_right = True
            # else:
            #     aim(1500, 1500, 1500, 1500)
            elif event.type == pygame.KEYUP:
                if event.key == 119:
                    forward = False
                if event.key == 97:
                    left = False
                if event.key == 115:
                    back = False
                if event.key == 100:
                    right = False
                if event.key == 105:
                    come_up = False
                if event.key == 106:
                    turn_left = False
                if event.key == 107:
                    come_dowm = False
                if event.key == 108:
                    turn_right = False
            pygame.display.update()
        if forward:
            print("forward")
            aim(1800, a, c, a)
        elif back:
            print("back")
            aim(1200, a, b, a)
        elif left:
            print("left")
            aim(a, a, a, b)
        elif right:
            print("right")
            aim(a, a, a, c)
        elif turn_left:
            print("turn_left")
            aim(a, b, a, a)
        elif turn_right:
            print("turn_right")
            aim(a, c, a, a)
        elif come_up:
            print("come_up")
            aim(c, a, a, a)
        elif come_dowm:
            print("come_dowm")
            aim(b, a, a, a)
        else:
            print(time.time())
            print("1")
            aim(a, a, a, a)
