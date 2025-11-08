"""用来接收不同类型的信息"""
# Import mavutil
from pymavlink import mavutil
import matplotlib.pyplot as plt

... # set up vehicle connection, request messages as required

message_types = {'VFR_HUD', 'RAW_IMU', 'ATTITUDE', 'VIBRATION'}  # 把要获取消息的类型添加到字典中

def handle_VFRHUD_message(msg):
    """处理VFR_HUD类型的消息"""
    global VFRHUD_ParamsList
    groundspeed, heading, alt = msg.groundspeed, msg.heading, msg.alt
    VFRHUD_ParamsList=[groundspeed, heading, alt]
    # print('VFR_HUD', groundspeed, heading, alt)
    return VFRHUD_ParamsList

def handle_RAWIMU_message(msg):
    """处理RAW_IMU类型的消息"""
    global RAWIMU_ParamsList
    xacc, yacc, zacc = msg.xacc, msg.yacc, msg.zacc
    # print('RAW_IMU', xacc, yacc, zacc)
    RAWIMU_ParamsList=[xacc, yacc, zacc]
    return RAWIMU_ParamsList

def handle_ATTITUDE_message(msg):
    """处理ATTITUDE类型的消息"""
    global ATTITUDE_ParamsList
    roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed
    # print('ATTITUDE', roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
    ATTITUDE_ParamsList=[roll, pitch, yaw, rollspeed, pitchspeed, yawspeed]
    return ATTITUDE_ParamsList

def handle_VIBRATION_message(msg):
    """处理VIBRATION类型的消息"""
    global VIBRATION_ParamsList
    vibration_x, vibration_y, vibration_z = msg.vibration_x, msg.vibration_y, msg.vibration_z
    # print('VIBRATION', vibration_x, vibration_y, vibration_z)
    VIBRATION_ParamsList=[vibration_x, vibration_y, vibration_z]
    return VIBRATION_ParamsList

message_handlers = {
    'VFR_HUD': handle_VFRHUD_message,
    'RAW_IMU': handle_RAWIMU_message,
    'ATTITUDE':handle_ATTITUDE_message,
    'VIBRATION':handle_VIBRATION_message
}

# def handle_message(message):
#     type_ = message.get_type()
#     if type_ in message_handlers:
#         message_handlers[type_](message)
#     else:
#         print(type_)


# def DOFilter_From_DifferentType():
#     """主函数"""
#     # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#     global master
#     message=master.recv_match(type=message_types, blocking=True)
#     type_=message.get_type()
#     if not message:
#         pass
#     if type_ in message_handlers:
#         # message_handlers[type_](message)
#         if type_=='VFR_HUD':
#             print(message_handlers[type_](message))
#             VFRHUD_ParamsList=message_handlers[type_](message)
#         if type_=='RAW_IMU':
#             print(message_handlers[type_](message))
#             RAWIMU_ParamsList=message_handlers[type_](message)
#         if type_=='ATTITUDE':
#             print(message_handlers[type_](message))
#             ATTITUDE_ParamsList=message_handlers[type_](message)
#         if type_=='VIBRATION':
#             print(message_handlers[type_](message))
#             VIBRATION_ParamsList=message_handlers[type_](message)

def DOFilter_From_DifferentType():
    """主函数"""
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # message_handlers[type_](message)
    global type_, msg, VFRHUD_ParamsList, RAWIMU_ParamsList, ATTITUDE_ParamsList, VIBRATION_ParamsList
    if type_=='VFR_HUD':
        # print(message_handlers[type_](msg))
        VFRHUD_ParamsList=message_handlers[type_](msg)
    if type_=='RAW_IMU':
        # print(message_handlers[type_](msg))
        RAWIMU_ParamsList=message_handlers[type_](msg)
    if type_=='ATTITUDE':
        # print(message_handlers[type_](msg))
        ATTITUDE_ParamsList=message_handlers[type_](msg)
    if type_=='VIBRATION':
        # print(message_handlers[type_](msg))
        VIBRATION_ParamsList=message_handlers[type_](msg)
    AllROV_ParamsList=[VFRHUD_ParamsList,RAWIMU_ParamsList,ATTITUDE_ParamsList,VIBRATION_ParamsList]
    print(AllROV_ParamsList)


if __name__=='__main__':
    print(11111111111)
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    print(2222222222)
    while 1:
        msg=master.recv_match(type=message_types, blocking=True)
        print(33333333333333)
        type_=msg.get_type()
        if type_ in message_handlers:
            DOFilter_From_DifferentType()
        else:
            continue
