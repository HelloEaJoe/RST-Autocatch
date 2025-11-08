# Import mavutil
from pymavlink import mavutil
import time




# Create the connection
master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()


# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
#初始pwm设置为1500
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
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


def static():
    set_rc_channel_pwm(1,1500)
    set_rc_channel_pwm(2,1500)
    set_rc_channel_pwm(3,1500)
    set_rc_channel_pwm(4,1500)
    set_rc_channel_pwm(5,1500)
    set_rc_channel_pwm(6,1500)
    
def aim(pwm1,pwm2,pwm3):
    #上浮下潜
    set_rc_channel_pwm(3,pwm1)#上浮下潜
    set_rc_channel_pwm(6,pwm2)#左拐右拐
    set_rc_channel_pwm(5,pwm3)#前进后退
    set_rc_channel_pwm(4,1500)
    set_rc_channel_pwm(1,1500)
    set_rc_channel_pwm(2,1500)
    
    #set_rc_channel_pwm(9, 1100)
    #set_rc_channel_pwm(10, 1100)

def Heng(pwm):
    #上浮下潜
    set_rc_channel_pwm(3,1500)#上浮下潜
    set_rc_channel_pwm(4,1500)#左拐右拐
    set_rc_channel_pwm(5,1500)#前进后退
    set_rc_channel_pwm(6,pwm)#HENG
    set_rc_channel_pwm(1,1500)
    set_rc_channel_pwm(2,1500)

# def lights_1_level(pwm):
#     set_rc_channel_pwm(9, pwm)
   
# def lights_2_level(pwm):
#     set_rc_channel_pwm(10, pwm)

def turn():
    aim(1500,1625,1530)
    time.sleep(1)

def turn_depth():
    aim(1410,1600,1500)
    time.sleep(1)

def turn_first(pwm_1):
    aim(pwm_1,1600,1490)
    time.sleep(2)

def turn_depth_first(pwm_1):
    aim(pwm_1,1600,1500)
    time.sleep(2)

if __name__=='__main__':
    while True:
        aim(1550,1550,1550)
        print(111)
       





