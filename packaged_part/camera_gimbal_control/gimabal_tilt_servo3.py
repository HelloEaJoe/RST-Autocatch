"""使用3通道控制相机云台"""
import time
from pymavlink import mavutil

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
        0,            # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )

# Create the connection
master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
print(master)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# command servo_3 to go from min to max in steps of 50us, over 2 seconds


def control_servo_left(pwm):
    set_servo_pwm(3,pwm)
    
def control_servo_right(pwm):
    set_servo_pwm(2,pwm)
    
def open_claw():
    control_servo_left(950)
    control_servo_right(1050)
    
def close_claw():
    control_servo_left(1450)
    control_servo_right(500)

if __name__ == "__main__":
    #set_servo_pwm(1,500)
    # mix:500
    # man:2500
    #control_servo_left(1450) #950 - 1450
    #control_servo_right(500)#1050 - 500
    open_claw()
    time.sleep(0.7)
    close_claw()
    time.sleep(0.7)
    
#     second 750  third 800     1000 1050

    
