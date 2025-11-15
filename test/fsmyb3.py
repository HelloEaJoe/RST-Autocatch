from enum import Enum, auto
import time
import cv2
import numpy as np
from pymavlink import mavutil
import sys
import os

#舵机相关
from gpiozero import Servo
servo = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
def set_angle(angle):
    value = (angle - 90) / 90  # 转换为 [-1, 1]
    servo.value = max(-1, min(1, value))


# 修正项目路径（根据实际情况调整）
project_root = '/home/pi/autocatch/'  # 改为实际项目根目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, project_root)

# 导入项目模块
from packaged_part.move.move import aim, static, set_rc_channel_pwm, master
from packaged_part.move.arm_disarm import arm, disarm
from packaged_part.move.init import init
from packaged_part.pid_catalog_first.set_deepth import ud_pid
from packaged_part.pid_catalog_first.sheer_pid import pipe1_fast, pipe3
from packaged_part.camera_gimbal_control.gimabal_tilt_servo3 import open_claw, close_claw
from packaged_part.camera_gimbal_control.LED import green_blink, red_blink, blue_blink

# 导入深度读取模块
from gx_provincial.parameter.read_depth import read_depth_continual

# 导入球检测模块
from balltest import yolov5_lite

class State(Enum):
    """定义机器人的所有可能状态"""
    DIVE = auto()               # 1. 下潜
    FIND_BALL = auto()          # 2. 找球 (决策)
    SEARCH_SPIN = auto()        # 3. 原地搜索 (如果没找到)
    APPROACH = auto()           # 4. 靠近
    GRAB = auto()               # 5. 抓球
    ASCEND_FOR_SCORE = auto()   # 6. 上浮计分
    OPEN_CLAW_RELEASE = auto()  # 7. 开爪释放
    MOVE_AWAY = auto()          # 9. 移开
    STOP = auto()               # 10. 停止
    MVANDDIVE = auto()          # 11. 移开并下潜
    ASCEND_AND_DIVE = auto()    # 12. 上浮并下潜,APPROACH后调用

class RobotStateMachine:
    """阻塞式状态机模块。"""
    def __init__(self, work_depth = 0.4):
        self.WORK_DEPTH = work_depth
        self.current_state = State.DIVE
        # 初始化MAVLink连接
        print("使用已有的MAVLink连接...")
        self.master = master
        print("飞控连接成功")
        # 初始化飞控
        disarm()
        init()
        arm()
        print("飞控已解锁")
        # 初始化时打开爪子，确保初始为张开状态
        #open_claw()
        set_angle(55)
        print("爪子已张开")
        # 初始化摄像头
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        if not self.camera.isOpened():
            raise RuntimeError("无法打开摄像头")
        print("摄像头初始化成功")
        # 初始化YOLO球检测模型（修正路径）
        print("加载球检测模型...")
        # 直接使用绝对路径，避免拼接错误
        model_path = "/home/pi/autocatch/model/bestball320.onnx"
        names_path = "/home/pi/autocatch/YOLOv5Litemaster/cube.names"
        
        # 检查模型文件是否存在
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"模型文件不存在: {model_path}")
        if not os.path.exists(names_path):
            raise FileNotFoundError(f"类别名称文件不存在: {names_path}")
        
        self.ball_detector = yolov5_lite(
            model_pb_path=model_path,
            label_path=names_path,
            confThreshold=0.5,
            nmsThreshold=0.5,  # 与 balltest.py 保持一致
            objThreshold=0.5
        )

        # 检查模型是否加载成功
        if not hasattr(self.ball_detector, 'net') or self.ball_detector.net is None:
            raise RuntimeError("模型加载失败，ball_detector.net未初始化")

        print("球检测模型加载成功")
        print("ball_detector 状态:", self.ball_detector)
        print("ball_detector.net 状态:", self.ball_detector.net)
        # 计时器
        self.search_start_time = None
        self.approach_start_time = None
        self.SEARCH_TIMEOUT = 10  # 搜索超时10秒
        self.APPROACH_TIMEOUT = 15.0  # 靠近超时15秒，根据距离，需要调整，xyb
        self.refresh_interval = 0.1  # 图像刷新间隔，根据板子状况越小越好，需要调整，xyb
        # 目标跟踪
        self.target_ball = None  # 当前目标球
        self.target_lost_count = 0  # 目标丢失计数
        self.TARGET_LOST_THRESHOLD = 5  # 连续丢失5帧认为丢失
        self.detected_boxes = []  # 存储检测到的所有球
        self.GREEN_GRAB_LIMIT = 3  # 抓取绿球的最大次数
        self.green_grab_count = 0  # 已成功抓取的绿球次数
        print(f"状态机已初始化，起始状态: {self.current_state.name}")
        print("当前使用的推理后端:", "OpenVINO" if self.ball_detector.use_openvino else "ONNXRuntime")
        # 深度相关
        self.current_depth = 0.0
        print(f"状态机已初始化，起始状态: {self.current_state.name}")
        print("当前使用的推理后端:", "OpenVINO" if self.ball_detector.use_openvino else "ONNXRuntime")


    def run(self):
        print("状态机启动...")
        total_timer_start_time=time.time()
        try:
            # while self.current_state != State.STOP and (time.time()-total_timer_start_time <= 120):#定时器，测试用，xyb
            while self.current_state != State.STOP:
                if self.current_state == State.DIVE:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_dive()
                elif self.current_state == State.FIND_BALL:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_find_ball()
                elif self.current_state == State.SEARCH_SPIN:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_search()
                elif self.current_state == State.APPROACH:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_approach()
                elif self.current_state == State.GRAB:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_grab()
                elif self.current_state == State.ASCEND_FOR_SCORE:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_ascend_for_score()
                elif self.current_state == State.OPEN_CLAW_RELEASE:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_open_claw_release()
                elif self.current_state == State.MOVE_AWAY:
                    print(f"进入[{self.current_state.name}]状态")
                    self.current_state = self._state_move_away()
                elif self.current_state == State.MVANDDIVE:
                    print("进入[MVANDDIVE]状态")
                    self.current_state = self._state_mv_and_dive()
                elif self.current_state == State.ASCEND_AND_DIVE:
                    print("进入[ASCEND_AND_DIVE]状态")
                    self.current_state = self._state_ascend_and_dive()

        except KeyboardInterrupt:
            print("\n检测到手动停止 (Ctrl+C)...")
            self.current_state = State.STOP
        except Exception as e:
            print(f"\n!! 状态机遇到致命错误: {e} !!")
            self.current_state = State.STOP
        # 清理资源
        self._cleanup()
        print("状态机已停止。")
    
    def _cleanup(self):
        """清理资源"""
        print("清理资源...")
        try:
            static()  # 停止所有运动
            disarm()  # 上锁飞控
            self.camera.release()  # 释放摄像头
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"清理资源时出错: {e}")

    def _state_dive(self):
        """1. 下潜状态 (阻塞)
            只有在初始化的时候调用一次
        """
        self.test_depth_and_sink() # 测试下潜至工作深度
        print("任务：下潜至工作深度")
        #open_claw()  # 确保爪子张开
        set_angle(55)

        print("下潜完成，进入找球状态")
        return State.FIND_BALL
    
    def _state_find_ball(self):
        """2. 找球决策 (非阻塞)
            在1下潜4靠近、搜索等状态丢失目标、9移开、11移动并下潜后调用
        """

        print("任务：决策是否找到球")
        if self._select_ball():
            print("找到目标，进入靠近状态")
            return State.APPROACH
        else:
            print("未找到目标，进入搜索状态")
            return State.SEARCH_SPIN

    def _state_search(self):
        """3. 原地搜索 (阻塞)
            在2找球决策未找到目标后调用，有开爪
        """
        print("任务：原地旋转, 直到找到球或超时")
        self._start_search_timer()
        set_angle(55)
        while not self._select_ball():
            if self._check_spin_timeout():
                print("完成: 搜索超时，未找到球。")
                return State.MOVE_AWAY
            self._do_spin(self.refresh_interval)
        print("完成: 搜索时发现目标。")
        return State.APPROACH

    def _state_approach(self):
        """4. 靠近 (阻塞)
            在2找球决策或3搜索找到目标后调用
        """
        print("任务: 靠近目标，直到可以抓取")
        self._start_approach_timer()
        set_angle(55)
        while True:
            # 1. 检查是否丢失目标
            if self._check_target_lost():
                print("异常: 目标丢失，放弃靠近。")
                return State.FIND_BALL
            # 2. 检查是否超时
            if self._check_approach_timeout():
                print("异常: 靠近超时，放弃靠近。")
                return State.ASCEND_AND_DIVE
            # 3. 检查是否已到位
            if self._should_grab():
                print("完成: 到达抓取位置。")
                return State.GRAB        
            # 4. 执行PID控制
            print("使用靠近pid算法，靠近中...")
            self._do_approach_pid()
            
    def _state_grab(self):
        """5. 抓球 (阻塞)
            在4靠近到位后调用
        """
        self.test_depth_and_sink() # 测试下潜至工作深度
        print("任务: 合爪并确认是否抓到")
        start_time = time.time()
        while time.time() - start_time < 1.0:
            aim(1410, 1500, 1500)  # 抓球时保持悬停
        self._do_close_claw()
        self._register_successful_grab()
        print("完成: 确认抓到球。")
        return State.ASCEND_FOR_SCORE

    def _register_successful_grab(self):
        """记录成功抓取的目标类型"""
        if self.target_ball is None:
            return
        class_name = str(self.target_ball.get('class_name', '')).lower()
        if class_name == 'green' and self.green_grab_count < self.GREEN_GRAB_LIMIT:
            self.green_grab_count += 1
            if self.green_grab_count >= self.GREEN_GRAB_LIMIT:
                print("已成功抓取3次绿球，后续将忽略绿球目标。")
            else:
                print(f"成功抓取绿球 {self.green_grab_count}/{self.GREEN_GRAB_LIMIT} 次。")

    def _state_ascend_for_score(self):
        """6. 上浮计分 (阻塞)
            在5抓球后调用
        """
        print("任务: 上浮到计分深度")
        self._do_ascend()
        return State.OPEN_CLAW_RELEASE

    def _state_open_claw_release(self):
        """7. 开爪释放 (阻塞)
            在6上浮后调用"""
        print("任务: 开爪")
        self._do_open_claw()
        print("完成: 已开爪。")
        self._select_ball()  # 刷新目标状态
        return State.MVANDDIVE

    def _state_move_away(self):
        """9. 移开 (阻塞)
            在2找球决策超时后调用
        """
        print("任务: 前进一小段")
        start_move_away_time = time.time()
        while time.time() - start_move_away_time < 7.0: #前进时间，需要调整，xyb
            self._select_ball()  # 刷新目标状态
            self._do_move_forward(self.refresh_interval)
        print("完成: 已移开。")
        return State.FIND_BALL
    
    def _state_mv_and_dive(self,move_period=3.0): #前进时间，需要调整，xyb
        """11.移动并下潜状态 (阻塞)"""
        print("任务：移动并下潜至工作深度")
        set_angle(55)
        pwm_held = False
        start_mv_and_dive_time = time.time()
        while time.time() - start_mv_and_dive_time < move_period:
            self._do_move_forward(self.refresh_interval)
            self._select_ball()
            pwm_held = True
        print("移动完成，准备下潜")
        # while True:
        #     self._do_move_forward()
        #     pwm_held = True
        #     if pwm_held and (time.time() - start_time) >= 3.0:
        #         print("向前移动3秒，准备切换状态")
        #         break
        self.test_depth_and_sink() # 测试下潜至工作深度
        print("移动下潜完成，进入找球状态")
        return State.FIND_BALL
    
    def _state_ascend_and_dive(self):
        '''12.上浮并下潜状态 (阻塞),防止卡在褶皱上
            在4靠近超时后调用
        '''
        print("任务：上浮并下潜至工作深度")
        self._do_ascend(0.5)    # 上浮0.5秒，调整时间需要根据实际情况,xyb
        self.test_depth_and_sink() # 测试下潜至工作深度
        print("上浮并下潜完成，进入找球状态")
        return State.FIND_BALL


# 以下为状态机辅助函数实现
    def _do_sink(self,period=0,pwm1=1410):
        """执行下潜动作，支持动态调整PWM值"""
        aim(pwm1, 1500, 1500)  # 保持航向和不前进
        time.sleep(period)


    # 只展示关键修改部分，其他代码保持不变
    def _select_ball(self) -> bool:
        """使用YOLO检测球并选择目标，并显示实时画面和检测结果"""
        ret, frame = self.camera.read()
        if not ret:
            return False

        # 再次检查模型是否初始化成功
        if not hasattr(self.ball_detector, 'net') or self.ball_detector.net is None:
            raise RuntimeError("ball_detector 未正确初始化，无法运行推理")

        img, newh, neww, top, left = self.ball_detector.resize_image(frame)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)

        # ===== 修正推理调用方式（关键修改）=====
        if self.ball_detector.use_openvino:
            # OpenVINO 正确推理流程：通过InferRequest执行
            infer_request = self.ball_detector.net.create_infer_request()  # 创建推理请求
            infer_request.infer({self.ball_detector.openvino_input_name: blob})  # 执行推理
            # 获取输出结果（根据输出名称或索引）
            output_tensor = infer_request.get_output_tensor(0)  # 使用索引 0 获取输出张量
            outs = output_tensor.data.squeeze(axis=0)  # 处理输出格式
        else:
            # ONNXRuntime 推理流程
            outs = self.ball_detector.net.run(None, {self.ball_detector.net.get_inputs()[0].name: blob})[0].squeeze(axis=0)
        # =====================================

        row_ind = 0
        for i in range(self.ball_detector.nl):
            h, w = int(self.ball_detector.input_shape[0] / self.ball_detector.stride[i]), \
                int(self.ball_detector.input_shape[1] / self.ball_detector.stride[i])
            length = int(self.ball_detector.na * h * w)
            if self.ball_detector.grid[i].shape[2:4] != (h, w):
                self.ball_detector.grid[i] = self.ball_detector._make_grid(w, h)

            # 修正广播错误：确保grid和anchor_grid在运算前具有与outs切片兼容的形状
            grid_tiled = np.tile(self.ball_detector.grid[i], (self.ball_detector.na, 1))
            grid_reshaped = grid_tiled.reshape(outs[row_ind:row_ind + length, 0:2].shape)
            outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + grid_reshaped) * int(self.ball_detector.stride[i])
            
            anchor_grid_repeated = np.repeat(self.ball_detector.anchor_grid[i], h * w, axis=0)
            anchor_grid_reshaped = anchor_grid_repeated.reshape(outs[row_ind:row_ind + length, 2:4].shape)
            outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * anchor_grid_reshaped
            row_ind += length

        # 修复 grid 初始化，确保与 balltest.py 一致
        self.ball_detector.grid = [np.zeros(1)] * self.ball_detector.nl

        # 后续可视化和目标选择逻辑不变...
        frameHeight, frameWidth = frame.shape[:2]
        ratioh, ratiow = frameHeight / newh, frameWidth / neww

        self.detected_boxes = []
        for detection in outs:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > self.ball_detector.confThreshold and detection[4] > self.ball_detector.objThreshold:
                center_x = int((detection[0] - left) * ratiow)
                center_y = int((detection[1] - top) * ratioh)
                width = int(detection[2] * ratiow)
                height = int(detection[3] * ratioh)
                area = width * height

                self.detected_boxes.append({
                    'class_id': classId,
                    'class_name': self.ball_detector.classes[classId],
                    'confidence': float(confidence),
                    'x': center_x,
                    'y': center_y,
                    'width': width,
                    'height': height,
                    'area': area
                })

        # 可视化检测结果
        vis_frame = frame.copy()
        for box in self.detected_boxes:
            x, y, w, h = box['x'], box['y'], box['width'], box['height']
            cv2.rectangle(vis_frame, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 2)
            label = f"{box['class_name']}:{box['confidence']:.2f}"
            cv2.putText(vis_frame, label, (x - w // 2, y - h // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # print(f"检测结果: 类别={box['class_name']} 置信度={box['confidence']:.2f} 区域=({x},{y},{w},{h}) 面积={box['area']}")

        cv2.imshow('Ball Detection', vis_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('检测窗口已关闭')
            cv2.destroyAllWindows()

        if len(self.detected_boxes) > 0:
            green_limit_reached = self.green_grab_count >= self.GREEN_GRAB_LIMIT
            selectable_boxes = [
                box for box in self.detected_boxes
                if not (green_limit_reached and str(box.get('class_name', '')).lower() == 'green')
            ]
            if len(selectable_boxes) == 0:
                self.target_ball = None
                return False

            # 优先选择类别为 'pink' 的目标；若无则按面积最大
            preferred = [b for b in selectable_boxes if str(b.get('class_name', '')).lower() == 'pink']
            candidates = preferred if len(preferred) > 0 else selectable_boxes
            self.target_ball = max(candidates, key=lambda b: b['area'])
            self.target_lost_count = 0
            # print(f"检测到球: {self.target_ball['class_name']}, 位置:({self.target_ball['x']}, {self.target_ball['y']}), 面积:{self.target_ball['area']}")
            return True

        self.target_ball = None
        return False
    def _start_search_timer(self):
        """启动旋转超时计时"""
        self.search_start_time = time.time()
        
    def _check_spin_timeout(self) -> bool:
        """检查旋转是否超时"""
        if self.search_start_time is None:
            return False
        elapsed = time.time() - self.search_start_time
        return elapsed > self.SEARCH_TIMEOUT

    def _do_spin(self,spin_period=0.1):
        """执行原地旋转"""
        pwm1 = 1410  # 保持下潜
        pwm2 = 1670  # 右转,这个数和控制转多少圈有关，尽量比一圈多一点1670，如果觉得快可以尝试一下1660，需要调整，xyb
        pwm3 = 1500  # 不前进
        aim(pwm1, pwm2, pwm3)
        time.sleep(spin_period)

    def _start_approach_timer(self):
        """启动靠近超时计时"""
        self.approach_start_time = time.time()

    def _check_target_lost(self) -> bool:
        """检查目标是否丢失"""
        ret, frame = self.camera.read()
        if not ret:
            self.target_lost_count += 1
            return self.target_lost_count > self.TARGET_LOST_THRESHOLD
        
        if not self._select_ball():
            self.target_lost_count += 1
        else:
            self.target_lost_count = 0
        
        return self.target_lost_count > self.TARGET_LOST_THRESHOLD

    def _check_approach_timeout(self) -> bool:
        """检查靠近是否超时"""
        if self.approach_start_time is None:
            return False
        elapsed = time.time() - self.approach_start_time
        return elapsed > self.APPROACH_TIMEOUT

    def _should_grab(self) -> bool:
        """判断是否到位可以抓"""
        if self.target_ball is None:
            return False
        
        center_x = 320 / 2
        y_target = 320 * 3 / 4  # 图像下1/4处
        x_diff = abs(self.target_ball['x'] - center_x)
        y_diff = abs(self.target_ball['y'] - y_target)
        area = self.target_ball['area']

        if x_diff < 20 and y_diff < 20 :
            print(f"到达抓取位置！X误差:{x_diff:.1f}, Y误差:{y_diff:.1f}, 面积:{area}")
            return True
        return False

    def _do_approach_pid(self):
        """执行靠近PID控制"""
        if self.target_ball is None:
            print("目标丢失，无法执行靠近PID控制,退出靠近状态")
            self._do_sink(pwm1=1410) # 目标丢失时，保持下沉而不是悬停
            return
        # 垂直方向以轻微下沉(1410)为基准进行调整
        pwm1 = 1410
        
        center_x = 320 / 2
        center_y = 320 / 2
        
        error_x = (self.target_ball['x'] - center_x) / 320
        error_y = (self.target_ball['y'] - center_y) / 320
        
        pwm2 = int(pipe1_fast.calculate(0, error_x, 1500))
        
        area = self.target_ball['area']
        
        pwm3 = 1620 # 保持定速向前推进
        
        aim(pwm1, pwm2, pwm3)
    
    def _do_close_claw(self):
        """合爪，并在等待时刷新图像"""
        print("正在合爪...")
        # 闭爪计时
        set_angle(99)
        aim(1410, 1500, 1500)  # 合爪时保持悬停
        time.sleep(0.7)  # 等待爪子完全闭合
        print("合爪完成")
    
    def _do_ascend(self,ascend_period=0.7):
        """上浮"""
        pwm1 = 1600  # 上浮
        pwm2 = 1500  # 保持航向
        pwm3 = 1500  # 不前进
        aim(pwm1, pwm2, pwm3)
        time.sleep(ascend_period)  # 上浮一段时间
        
    def _do_open_claw(self):
        """开爪"""
        print("正在开爪...")
        set_angle(55)
        aim(1430, 1500, 1500)  # 开爪时保持悬停
        time.sleep(0.7)  # 等待爪子完全打开
        print("开爪完成")
        
    def _do_wait(self, seconds: float):
        """等待"""
        print(f"等待 {seconds} 秒...")
        time.sleep(seconds)

    def _do_move_forward(self,move_period=1.0):
        """前进一小段"""
        print(f"前进一小段，持续时间: {move_period} 秒...")
        pwm1 = 1410  # 保持下潜
        pwm2 = 1500  # 保持航向
        pwm3 = 1570  # 前进
        aim(pwm1, pwm2, pwm3)
        time.sleep(move_period)  # 确保有短暂的停止
        print("前进完成")

    def test_depth_and_sink(self):
        """测试下潜至工作深度"""
        pwm_held = False  # 添加标志位，表示是否保持当前 PWM 值
        while self._get_current_depth() < self.WORK_DEPTH:
            self._do_sink(0.1)
            pwm_held = True
        if pwm_held:
            print("保持当前下沉状态，PWM 值不变")

    def _get_current_depth(self) -> float:
        """从read_depth.py获取当前深度（通过AHRS2消息）"""
        try:
            depth_cm = read_depth_continual(self.current_depth)
            if depth_cm is not None:
                self.current_depth = depth_cm / 100.0
                print(f"当前深度: {self.current_depth:.2f}米 ({depth_cm:.1f}厘米)")
                return self.current_depth
            else:
                print("警告: 无法从AHRS2获取深度数据，使用上一次的值")
                return self.current_depth
        except Exception as e:
            print(f"获取深度数据错误: {e}")
            return self.current_depth
        
    
if __name__ == "__main__":
    robot_sm = RobotStateMachine(work_depth=0.68) # 需要调整，xyb
    robot_sm.run()
