"""
用pymavlink协议去解锁和锁上飞行器
"""
# Import mavutil
from pymavlink import mavutil
#解锁
def arm():
    # Create the connection
    master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
    #master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
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
    #master.motors_armed_wait()
    print('Armed!')

#上锁
def disarm():
    master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
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