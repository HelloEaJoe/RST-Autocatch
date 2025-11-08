"""
Example of how to filter for specific mavlink messages coming from the
autopilot using pymavlink.

Can also filter within recv_match command - see "Read all parameters" example
"""
# Import mavutil
from telnetlib import DO
from pymavlink import mavutil
import matplotlib.pyplot as plt
import time
# Create the connection
# From topside computer



def DONotFilter():
    """不对信息进行过滤的函数"""
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg: #会打印出所有msg
            # print("\n\n*****Got message: %s*****" % msg.get_type())
            print("Message: %s" % msg)




def DOFilter_From_SingleType(msg_type):    #msg_type是一个字符串类型的数据
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    x_time=[]
    y_xacc=[]
    y_yacc=[]
    y_zacc=[]
    plt.ion()
    while True:
        msg = master.recv_match()
        #print(msg)
        if not msg:
            continue
        if msg.get_type() == msg_type:
            #print("\n\n*****Got message: %s*****" % msg.get_type()) #会打印出消息的类型
            #print("Message: %s" % msg)  #会打印出进行上面一层过滤以后的所有msg
            print(msg)
            x_time.append(time.time())
            y_xacc.append(msg.xacc)
            y_yacc.append(msg.yacc)
            y_zacc.append(msg.zacc)
            plt.clf()              # 清除之前画的图
            plt.plot(x_time,y_xacc)        # 画出当前 ax 列表和 ay 列表中的值的图形
            plt.plot(x_time,y_yacc)
            plt.plot(x_time,y_zacc)
            plt.xlabel('time')
            plt.ylabel('IMU')
            plt.pause(0.00000001)         # 暂停一秒
            plt.ioff()             # 关闭画图的窗口



if __name__=='__main__':
    DOFilter_From_SingleType('RAW_IMU')
    # DO_NotFilter()
