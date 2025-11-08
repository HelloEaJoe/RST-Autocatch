"""读取ROV罗盘的偏航角度"""
# Import mavutil
from pymavlink import mavutil


# Create the connection
# From topside computer

def read_attitude_continual(Boolean):
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    while Boolean:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            print(msg.heading)


if __name__ == '__main__':
    read_attitude_continual(True)
