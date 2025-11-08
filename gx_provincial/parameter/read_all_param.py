"""读出ROV所有参数并写入到param_data.log文件中"""

# Disable "Broad exception" warning
# pylint: disable=W0703

import time
import sys

# Import mavutil
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Request all parameters
master.mav.param_request_list_send(
    master.target_system, master.target_component
)
file=open('param_data.log',mode='a',encoding="utf-8")
file.write('日志记录的时间: ')
file.write(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
file.write('\n')
while True:
    #time.sleep(0.01)
    try:
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'],
                                       message['param_value']))
        file.write('name: %s\tvalue: %d' % (message['param_id'],message['param_value']))
        file.write('\n')
    except Exception as error:
        print(error)
        sys.exit(0)