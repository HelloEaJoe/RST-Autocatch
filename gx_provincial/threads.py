import threading
import time
from multiprocessing import Process, Queue
from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *
import cv2
# from pymavlink import mavutil
from parameter.read_depth import *
from parameter.real_read_yaw import *
from pid_catalog_first.sheer_pid import *
# from move.move import *
# from move.init import *
# from move.arm_disarm import arm,disarm

#
# result_data = {
#     'object_exists': False,
#     'object_left': None,
#     'object_top': None,
#     'object_width': None,
#     'object_height': None,
#     'center_x': None,
#     'center_y': None,
#     'object_area': None,
#     'robot_yaw': None,
#     'depth1': None
# }

# lock = threading.Lock()

frame_queue = Queue()
depth_queue = Queue()


# def run_pwm_function(duration, pwm1, pwm2, pwm3, pwm4):
#     start_time_1 = time.time()
#     end_time_1 = start_time_1 + 0.1
#     while True:
#         depth1 = read_depth_continual(True)
#         robot_yaw = read_yaw_continually(True)
#         if end_time_1  - start_time_1< duration:
#             aim(pwm1, pwm2, pwm3)
#             Heng(pwm4)
#             end_time_1 = time.time()
#         else:
#             break


# def read_depth_continual():
#     # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#     master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
#     while True:
#         msg = master.recv_match()
#         if not msg:
#             continue
#         if msg.get_type() == 'AHRS2':
#             # print("\nAs dictionary: %s" % msg.to_dict())
#             depth =  -msg.altitude * 100
#             print("当前深度：", depth)
#             depth_queue.put(depth)
#             # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
#             # print("\nSystem status: %s" % msg.system_status)
#     # msg = master.recv_match()
#     # if msg:
#     #     if msg.get_type() == 'AHRS2':
#     #         print("当前深度：", -msg.altitude*100)
#     #         depth = -msg.altitude*100
#     #         print('receive')
#     # return depth


def process_frame():
    global object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area, result
    result = []
    object_exists = False
    time_before = time.time()
    # 电脑的模型路径
    model_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\weights\best_1.onnx"
    label_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\data\xian.yaml"
    # pi的模型路径
    # model_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/weights/best_1.onnx"
    # label_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/data/xian.yaml"
    net = yolov5_lite(model_path, label_path, confThreshold=0.5, nmsThreshold=0.5)

    cap = cv2.VideoCapture(0)
    # ret, frame = cap.read()
    while True:
        time_before = time.time()
        ret, frame = cap.read()
        process_frame = net.detect(frame)
        rectangle_coordinate = net.get_rectangle_coordinate()
        owning_classes = net.get_owning_classes()
        center = net.get_center()
        area = net.get_area()
        time_later = time.time()

        for i in range(len(rectangle_coordinate)):
            if owning_classes[i] == 'redball':
                object_exists = True
                object_left, object_top, object_width, object_height = rectangle_coordinate[i]
                center_x, center_y = center[i]
                object_area = area[i]
                result = [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
                          object_area]
                frame_queue.put(result)
                break
        # time_later = time.time()
        time_cost = time_later - time_before
        print(time_cost)
        cv2.imshow('frame', process_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

        # print(result)

    # result = (object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area)
    # frame_queue.put(result)
    #         result = frame_queue.get()


# def control():
#     while True:
#         [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,object_area] = frame_queue.get()
#         # [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,object_area] = result
#         depth = depth_queue.get()
#
#         if object_exists is not True:
#
#             pwm1 = int(depth_pid.calculate(60, depth, 1495))
#             aim(pwm1, 1500, 1550)
#
#         else:
#
#             if object_area <= 0.8:
#                 pwm1 = int(depth_pid.calculate(60, depth, 1495))
#                 pwm2 = int(yaw_pid.calculate(0.5, center_x, 1495))
#                 aim(pwm1, pwm2, 1550)
#
#             else:
#
#                 # 撞球动作
#                 run_pwm_function(2, int(depth_pid.calculate(60, depth, 1495)),
#                                  1500, 1550, 1500)
#                 # 转换状态到patroll_lines_to_next
#
#                 print(f"state changes to patroll_lines_to_next")
#                 break


def test_print():
    depth = 0
    while True:
        if not frame_queue.empty():
            result = frame_queue.get()  # 从队列中获取结果
            # 注意：由于break语句的存在，这个while循环可能会在某个点退出
        # 如果您希望线程继续运行以等待新的帧或执行其他任务，则需要添加额外的逻辑来处理这种情况


if __name__ == '__main__':
    # 启动线程
    frame_thread = Process(target=process_frame)
    # depth_thread = Process(target=read_depth_continual)
    # test_thread = Process(target=test_print)
    # control_thread = threading.Thread(target=control)
    frame_thread.start()
    # test_thread.start()
    # control_thread.start()
    # frame_thread.join()
