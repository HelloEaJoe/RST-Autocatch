import threading
import time
from multiprocessing import Event, Process, Queue
from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *
import cv2
from pymavlink import mavutil
from parameter.read_depth import *
from parameter.real_read_yaw import *
from pid_catalog_first.sheer_pid import *
from move.move import *
from move.init import *
from move.arm_disarm import arm, disarm

frame_queue = Queue(maxsize=1)
depth_queue = Queue(maxsize=1)



def hit():
    stop_event = Event()
    # frame_thread = Process(target=process_frame, args=('redball',))
    frame_thread = Process(target=process_frame, args=('redball',stop_event))
    frame_thread.start()
    hit_thread = Process(target=control_hit,args=(stop_event,))
    hit_thread.start()
    hit_thread.join()
    # frame_thread.join()


def door(isHigh):
    print("进入door")
    arg=('line','blueball')
    frame_thread = Process(target=process_frame, args=('door',))
    frame_thread.start()
    door_thread = Process(target=control_door, args=(isHigh,))
    door_thread.start()
    door_thread.join()
    # frame_thread.join()


def line():
    stop_event = Event()
    arg=('line','blueball')
    frame_thread = Process(target=process_frame, args=(arg,stop_event))
    frame_thread.start()
    line_thread = Process(target=control_line,args=(stop_event,))
    line_thread.start()
    print("line开始")
    line_thread.join()
    print("line结束")
    time.sleep(1)
    # frame_thread.join()
    print("frame结束")



def grip_ball():
    frame_thread = Process(target=process_frame, args=('pinkball',))
    frame_thread.start()
    grip_thread = Process(target=control_small_ball)
    grip_thread.start()
    grip_thread.join()
    frame_thread.join()


def loose_ball():
    frame_thread = Process(target=process_frame, args=('busket',))
    frame_thread.start()
    loose_thread = Process(target=control_bucket)
    loose_thread.start()
    loose_thread.join()
    frame_thread.join()



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


def run_pwm_function(duration, pwm1, pwm2, pwm3, pwm4):
    start_time_1 = time.time()
    end_time_1 = start_time_1 + 0.1
    while True:
        depth1 = depth_queue.get()
        robot_yaw = read_yaw_continually(True)
        if end_time_1 - start_time_1 < duration:
            aim(pwm1, pwm2, pwm3, pwm4)

            end_time_1 = time.time()
        else:
            break


def continue_read_act(duration, target_pwm1, target_pwm2, target_pwm4, pwm3):
    start_time_1 = time.time()
    end_time_1 = start_time_1 + 0.1
    while True:
        if end_time_1 - start_time_1 < duration:
            depth = depth_queue.get()
            center_x = frame_queue.get()[5]
            pwm1 = int(depth_pid.calculate(target_pwm1, depth, 1495))
            if target_pwm2 != False:
                pwm2 = int(yaw_pid.calculate(target_pwm2, center_x, 1495))
            else:
                pwm2 = 1500
            if target_pwm4 != False:
                pwm4 = int(heng_pid.calculate(target_pwm4, center_x, 1495))
            else:
                pwm4 = 1500
            # print(pwm1)
            aim(pwm1, pwm2, pwm3, pwm4)

            # Heng(pwm4)

            end_time_1 = time.time()
        else:
            break


def read_depth_continual():
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'AHRS2':
            # depth = -msg.altitude * 10
            if not depth_queue.empty():
                depth_queue.get()
            depth_queue.put(-msg.altitude * 10)


def get_line_center(frame):

    height, width, _ = frame.shape

    # 计算高度的一半附近的行范围
    row_start = height // 2 - 10  # 取中间行的上方10行
    row_end = height // 2 + 10  # 取中间行的下方10行

    middle_region = frame[row_start:row_end, :, :]

    # opencv里读图是BGR！！！！！！！！！！！！！！！！
    low_bgr = np.array([40,50,170])
    high_bgr = np.array([145, 175, 105])
    mask = cv2.inRange(middle_region, lowerb=low_bgr, upperb=high_bgr)

    coords = np.column_stack(np.where(mask > 0))
    # 如果找到保留的区域，计算中点
    if coords.size > 0:
        center = np.mean(coords, axis=0).astype(int)
        # 在原图上绘制中点
        cv2.circle(frame, (center[1], center[0] + row_start), 5, (0, 255, 0), -1)  # 在原图上绘制中点

        return center[1]
    return False


def grip():
    print('grip')
    set_servo_pwm(2, 1050)


def loose():
    print('loose')
    set_servo_pwm(2, 1800)


def process_frame(args,stop_event):
    global object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area, result
    result = []
    # print(1)
    print("图像开始")
    # time_before = time.time()
    # # 电脑的模型路径
    # model_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\weights\best_1.onnx"
    # label_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\data\xian.yaml"
    # pi的模型路径
    model_path = "/home/pi/Desktop/xian_auto_1/gx_provincial/YOLOv5Litemaster/weights/best_1.onnx"
    label_path = "/home/pi/Desktop/xian_auto_1/gx_provincial/YOLOv5Litemaster/data/xian.yaml"
    net = yolov5_lite(model_path, label_path, confThreshold=0.7, nmsThreshold=0.5)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] VideoCapture not opened. Exiting process_frame.")
        return

    frame_count = 0
    process_frame_img = None
    while True:
        if stop_event is not None and stop_event.is_set():
            print("[INFO] stop_event set, exiting process_frame loop.")
            break
        object_exists = False
        time_before = time.time()
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[ERROR] cap.read() failed or frame is None, breaking loop.")
            break
        if not isinstance(frame, np.ndarray):
            print("[ERROR] frame is not a numpy array, skipping.")
            continue
        if frame.size == 0:
            print("[ERROR] frame size is 0, skipping.")
            continue

        frame_count += 1
        if frame_count % 4 == 0:
            try:
                process_frame_img = net.detect(frame)
                if process_frame_img is None:
                    print("[ERROR] net.detect returned None, skipping.")
                    continue
            except Exception as e:
                print(f"[ERROR] net.detect exception: {e}")
                continue
            rectangle_coordinate = net.get_rectangle_coordinate()
            owning_classes = net.get_owning_classes()
            center = net.get_center()
            area = net.get_area()
            time_later = time.time()

            for i in range(len(rectangle_coordinate)):
                if owning_classes[i] == args:
                    object_exists = True
                    object_left, object_top, object_width, object_height = rectangle_coordinate[i]
                    center_x, center_y = center[i]
                    object_area = area[i]
                    result = [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
                            object_area, frame]
                    if not frame_queue.full():
                        frame_queue.put(result)
                    else:
                        print("[WARNING] frame_queue is full, skipping put.")

            if not object_exists:
                result = [object_exists, 0, 0, 0, 0, 0.5, 0.5, 0, frame]
            if not frame_queue.empty():
                frame_queue.get()

            print(f"[INFO] Frame time: {time_later - time_before:.4f}s, shape: {frame.shape if hasattr(frame, 'shape') else 'unknown'}")

            try:
                if process_frame_img is not None and isinstance(process_frame_img, np.ndarray) and process_frame_img.size > 0:
                    cv2.imshow('frame', process_frame_img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("[INFO] Quit key pressed, breaking loop.")
                        break
                else:
                    print("[WARNING] process_frame_img invalid for imshow.")
            except Exception as e:
                print(f"[ERROR] imshow/waitKey exception: {e}")
                break
        # 如果不是第3帧，依然要显示原始frame，避免窗口卡死
        else:
            try:
                if frame is not None and isinstance(frame, np.ndarray) and frame.size > 0:
                    cv2.imshow('frame', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("[INFO] Quit key pressed, breaking loop.")
                        break
            except Exception as e:
                print(f"[ERROR] imshow/waitKey exception (non-infer frame): {e}")
                break
    cap.release()
    cv2.destroyAllWindows()

    # print(result)

    # result = (object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area)
    # frame_queue.put(result)
    #         result = frame_queue.get()


def change_mode(index):

    if not (frame_queue.empty()):
        [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
         object_area,frame] = frame_queue.get()
    # [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,object_area] = result
    if not depth_queue.empty():
        depth = depth_queue.get()

    # 转换模式主要是为了调深度

    if index == 'line to hit':
        print('line to hit')

        # 这是需要调参的，撞球的深度，或者也可以对着
        big_ball_depth = 2.6

        continue_read_act(3, big_ball_depth, False, False, 1420)

    if index == 'hit to line':
        print('hit to line')

        # 设定好
        line_depth = 2.6

        continue_read_act(3, line_depth, False, False, 1420)

    if index == 'line to door':
        print('line to door')

    if index == 'door to line':
        print('door to line')

    if index == 'line to grip':
        print('line to grip')

    if index == 'grip to loose':
        print('grip to loose')

    if index == 'loose to back':
        print('loose to back')


def control_hit(stop):

    object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area = False, 0, 0, 0, 0, 0.5, 0.5, 0
    # depth = 1.5
    depth = 1.0
    object_exists=False
    hit_depth = 2
    # hit_depth = 6.2
    print("xiancheng2")
    while True:
        # print("进来了")
        # aim(1510,1500,1500,1500)
        # 有没有检测到东西
        pwm2=1500
        if not (frame_queue.empty()):
            [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
             object_area,frame] = frame_queue.get()
        # [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,object_area] = result
        if not depth_queue.empty():
            depth = depth_queue.get()
            # print(depth)
            # print("1111")

        pwm1 = int(line_depth_pid.calculate(hit_depth, depth, 1410))
        # pwm1 = int(up_down.calculate(center_y,0.5,  1430))
        # print(center_y)
        if object_exists is not True:
            # pwm1 = int(depth_pid.calculate(hit_depth, depth, 1365))
            # print(pwm1)
            # print("2222")
            aim(pwm1, 1500, 1580, 1500)
            # aim(pwm1, 1500, 1550, 1500)

            continue

        if object_exists is True:

            # print("object Area:" + str(object_area))
            if object_area <= 0.4:
                # print('areasmall')
                # pwm1 = int(depth_pid.calculate(hit_depth, depth, 1365))
                pwm2 = int(ball_yaw_pid.calculate(0.5, center_x, 1500))
                print(center_x)
                print(f"pwm2是", pwm2)
                aim(pwm1, pwm2, 1580, 1500)
                # aim(pwm1, 1500, 1550, 1500)

                continue

            else:
                print('RUSH!!!!')
                print(f"state changes to patroll_lines_to_next")
                break

           
        # start_time = time.time()
        # end_time = time.time() + 0.1
        # while end_time - start_time < 2:
        #     aim(pwm1, 1400, 1500, 1500)
        #     end_time = time.time()
        # while True:
        #     aim(pwm1, 1500, 1580, 1500)


def control_door(isHigh):

    object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area, frame = False, 0, 0, 0, 0, 0.5, 0.5, 0, False
    depth = 1.5
    rush_flag=False

    if isHigh== True:
        target_depth = 4.1
    else:
        # target_depth = 5.0
        target_depth = 5.0


    while True:
        # 有没有检测到东西
        if not (frame_queue.empty()):
            [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
             object_area,frame] = frame_queue.get()
        # [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,object_area] = result
        if not depth_queue.empty():
            depth = depth_queue.get()
            # print(depth)
            # print(1111)

        if object_exists is not True:
            pwm1 = int(depth_pid.calculate(target_depth, depth, 1365))

            aim(pwm1, 1500, 1560, 1500)
        if object_exists:
            print("door_True")
            start_time = time.time()
            while True:

                if not object_exists:
                    break

                depth = depth_queue.get()
                [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
                 object_area,frame] = frame_queue.get()

                pwm1 = int(depth_pid.calculate(target_depth, depth, 1000))
                print(pwm1)
                pwm2 = int(yaw_pid.calculate(0.5, center_x, 1495))
                aim(pwm1, pwm2, 1560, 1500)

                end_time = time.time()

                if (end_time - start_time >= 4) and (object_area >= 0.4):
                    continue_read_act(5, target_depth, False, False, 1560)
                    print('RUSH!!!!!!!!!')
                    rush_flag = True
                    break

            if rush_flag:
                print('door done')
                break


def control_line(stop_event):
    object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area, frame = False, 0, 0, 0, 0, 0.5, 0.5, 0, False
    depth = 1.5
    line_depth = 30.0
    count = 0
    ini_time=time.time()
    while True:
        if not (frame_queue.empty()):
            [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
             object_area, frame] = frame_queue.get()
        if not depth_queue.empty():
            depth = depth_queue.get()
            print(depth)

        pwm1 = int(line_depth_pid.calculate(line_depth, depth, 1410))
        # print(pwm1)

        # 没物体就先找
        if not object_exists:
            aim(pwm1, 1500, 1550, 1500)
            object_exists=False
            line_center=False      
        # 有物体
        else:
            # 对准物体
            # print(center_x)
            # pwm2 = int(line_yaw_pid.calculate(0.5, center_x, 1495))
            pwm2 = int(line_yaw_pid.calculate(50, center_x*100, 1500))


            # print(f"pwm2__为{pwm2}")
            line_center = get_line_center(frame)
            # print(line_center)
            # # 巡线
            if line_center != False:
                # pwm2 = int(line_yaw_pid.calculate(0.5, line_center, 1495))
                pwm2 = int(line_yaw_pid.calculate(50, line_center*100, 1500))
                # if pwm2==1500 and :
                #      aim(pwm1,1500,1550,1500)
                #      break
                # print(f"pwm2为{pwm2}")
                count += 1
                # print(object_exists)
            
            aim(pwm1, pwm2, 1550, 1500)
            # aim(pwm1, 1500, 1550, 1500)

        if  time.time()>ini_time+10:
            aim(pwm1,1500,1550,1500)
            print(11111111111111111111)
            # stop_event.set()
            break
        # if depth>60 and not center_x:
        #     aim(pwm1,1500,1550,1500)
        #     print(111)
        #     break
        continue


def control_small_ball():

    # object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area,
    # frame = False, 0, 0, 0, 0, 0.5, 0.5, 0, False

    depth = 1.5
    grip_depth = 6.0

    rush_flag = False
    while True:

        if not (frame_queue.empty()):
            [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
             object_area, frame] = frame_queue.get()
        if not depth_queue.empty():
            depth = depth_queue.get()

        pwm1 = int(depth_pid.calculate(grip_depth, depth, 1495))

        # 没找到物体
        if not object_exists:
            aim(pwm1, 1500, 1550, 1500)

        # 找到了
        else:
            print("bucket_True")
            start_time = time.time()
            while True:

                if not object_exists:
                    break

                depth = depth_queue.get()
                [object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area,
                 frame] = frame_queue.get()
            # 调整位置，让球在屏幕的大概下1/4（从上到下看是0.75）处
            ball_top = 0.75 # 球距离顶部
            pwm1 = int(depth_pid.calculate(ball_top, center_y, 1495))
            pwm2 = int(yaw_pid.calculate(0.5, center_x, 1495))
            aim(pwm1, pwm2, 1500, 1500)
            end_time = time.time()

            # 如果面积达到，并且调稳了，就可以写死去抓
            if (end_time - start_time >= 5) and (object_area >= 0.4):
                # 抓（我不知道是什么函数，先写了个grip空的放在这里）
                print('GRIP!!!!!!!!!')
                grip()


def control_bucket():
    object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area, frame = False, 0, 0, 0, 0, 0.5, 0.5, 0, False
    depth = 1.5

    bucket_depth = 6.0

    while True:

        if not (frame_queue.empty()):
            [object_exists, object_left, object_top, object_width, object_height, center_x, center_y,
             object_area,frame] = frame_queue.get()
        if not depth_queue.empty():
            depth = depth_queue.get()

        pwm1 = int(depth_pid.calculate(bucket_depth, depth, 1495))

        # 没找到东西
        if object_exists is not True:
            aim(pwm1, 1500, 1550, 1500)

        # 找到了
        if object_exists:
            print("bucket_True")
            start_time = time.time()
            while True:

                if not object_exists:
                    break

                depth = depth_queue.get()
                [object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area,
                 frame] = frame_queue.get()

                pwm1 = int(depth_pid.calculate(4.0, depth, 1495))
                pwm2 = int(yaw_pid.calculate(0.5, center_x, 1495))
                aim(pwm1, pwm2, 1500, 1500)

                end_time = time.time()

                # 如果面积达到，并且调稳了，就可以冲框，接下来全部写死
                if (end_time - start_time >= 5) and (object_area >= 0.4):

                    # 花两秒抬高
                    print('UP!!!!!!!!!')
                    run_pwm_function(2, 1500, 1500, 1500, 1500)

                    # 花两秒冲进去
                    print('RUSH!!!!!!!!!')
                    run_pwm_function(2, 1360, 1500, 1600, 1500)

                    # 松爪（我不知道是什么函数，先写了个loose空的放在这里）
                    print('LOOSE!!!!!!!!!')
                    loose()

                    break

def control_line1(stop_event):
    # 初始化变量（注意：frame初始化为None更合理，避免类型错误）
    object_exists, object_left, object_top, object_width, object_height = False, 0, 0, 0, 0
    center_x, center_y, object_area, frame = 0.5, 0.5, 0, None
    depth = 1.5
    line_depth = 63
    count = 0
    # 新增：用于记录最近5次object_exists的状态（前2次+后3次）
    exists_history = []  # 存储布尔值，True表示存在，False表示不存在
    has_10_true = False 

    while True:
        # 读取最新帧数据
        if not frame_queue.empty():
            [object_exists, object_left, object_top, object_width, object_height, 
             center_x, center_y, object_area, frame] = frame_queue.get()
        # 读取最新深度数据
        if not depth_queue.empty():
            depth = depth_queue.get()

        # 计算深度控制PWM
        pwm1 = int(line_depth_pid.calculate(line_depth, depth, 1410))

        # 记录当前object_exists状态（转为布尔值），并只保留最近5条记录
        current_exists = object_exists # 确保是布尔值
        exists_history.append(current_exists)


        # 无目标时的控制逻辑
        if not object_exists:
            aim(pwm1, 1500, 1540, 1500)
            current_exists=False
            # aim(1500, 1500, 1560, 1500)
        # 有目标时的控制逻辑
        else:
            # 计算偏航控制PWM（基于center_x）
            pwm2 = int(line_yaw_pid.calculate(50, center_x*100 , 1500))
            print(f"pwm2__为{pwm2}")
            aim(pwm1, pwm2, 1540, 1500)
        # # 2. 判断是否满足“前10次全为True”（仅在未满足标志时检查）
        # if not has_10_true:
        #     # 当历史记录长度达到10时，检查前10次是否全为True
        #     if len(exists_history) >= 5:
        #         # 取前10次状态，判断是否全为True
        #         first_10 = exists_history[:5]
        #         if all(first_10):  # all()：所有元素为True则返回True
        #             has_10_true = True  # 标记已满足前10次全为True

        # # 3. 核心逻辑：前10次全为True，且当前循环为False时，输出“好的”
        # if has_10_true and not current_exists:
        #         print("好的")  # 满足条件时输出
        # current_exists = bool(object_exists)  # 确保是布尔值
        # exists_history.append(current_exists)
        # if len(exists_history) > 5:  # 只保留最近5次状态
        #     exists_history.pop(0)

        # # 核心判断：前2次为True，后3次为False（需满足历史记录长度为5）
        # if len(exists_history) == 5:
        #     # 前两次（索引0、1）为True，后三次（索引2、3、4）为False
        #     if (exists_history[0] and exists_history[1] and 
        #         not exists_history[2] and not exists_history[3] and not exists_history[4]):
        #         print("好的")  # 满足条件时输出
        #         # while 1:
        #         #     aim(pwm1,1502,1570,1500)
        #         stop_event.set()
        #         break
        #原有退出条件
        if depth > 60 and not center_x:
            aim(pwm1, 1500, 1550, 1500)
            print(111)
            break
        
        continue
def con(stop_event):
    last=time.time()
    while time.time()<last+2:
        aim(1500,1510,1500,1500)
    stop_event.set()
    print(stop_event)


if __name__ == '__main__':
    global object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area, frame
    (object_exists, object_left, object_top, object_width, object_height, center_x, center_y, object_area,
     frame) = False, 0, 0, 0, 0, 0.5, 0.5, 0, False
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    disarm()
    init()
    arm()
    # 启动检测图像和检测深度的进程
    depth_thread = Process(target=read_depth_continual)
    depth_thread.start()

   
    hit()
    # line()
    # door(True)
    # line()
    # door(False)
    # line()
    # door(True)
    # line()
    # door(False)
    # line()
    # grip_ball()
    # loose_ball()
    # a =0
    # while 1:
        # aim(1500,1500,1580,1500)
        
        
    #     # print(a)
  
    # while a<20000:
        
    #     aim(1500,1500,1580,1500)
    #     a+=1
    # while a<30000:

    #     aim(1500,1500,1620,1500)
    #     a+=1
    #     # print(pwm2)
    # door(isHigh=True)
    