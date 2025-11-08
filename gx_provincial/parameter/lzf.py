import math
import time

from pid_catalog.yaw_pid import *
from move.arm_disarm import *
import os
from parameter.read_depth import *
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst
file_path = "read_target_param"

from ..v5lite_s.yolov5lite import *
from ..v5lite_s.yolov5 import *
import cv2

def overwrite_line(file_path, line_number, new_data):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    if line_number < 0 or line_number >= len(lines):
        # print(0)
        return False

    lines[line_number - 1] = new_data + '\n'
    print(lines)
    with open(file_path, 'w') as file:
        # print(1)
        file.writelines(lines)

    return True


def read_target_param(Boolean):
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
    # master.wait_heartbeat()
    # print(111)
    # video = Video()
    while Boolean:

        msg = master.recv_match()
        if video.frame_available():
            frame = video.frame()
            cv2.imwrite("video.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 30])

        if not msg:
            #print(2)
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            #print(3)
            # 线速度.x,y,z
            vx = str(msg.vx)
            vy = str(msg.vy)
            vz = str(msg.vz)
            vx_y_z = str(math.sqrt(msg.vx * msg.vx + msg.vy * msg.vy + msg.vz * msg.vz))
            overwrite_line(file_path, 1, vx)
            overwrite_line(file_path, 2, vy)
            overwrite_line(file_path, 3, vz)
            overwrite_line(file_path, 35, vx_y_z)
        elif msg.get_type() == 'ATTITUDE':
            # print(4)
            # 角速度x,y,z
            rollspeed = str(msg.rollspeed)
            pitchspeed = str(msg.pitchspeed)
            yawspeed = str(msg.yawspeed)
            # 翻滚角 偏航角 俯仰角
            roll = str(msg.roll)
            pitch = str(msg.pitch)
            yaw = str(msg.yaw)
            time_boot_ms = str(msg.time_boot_ms)
            overwrite_line(file_path, 4, rollspeed)
            overwrite_line(file_path, 5, pitchspeed)
            overwrite_line(file_path, 6, yawspeed)
            overwrite_line(file_path, 13, roll)
            overwrite_line(file_path, 14, pitch)
            overwrite_line(file_path, 15, yaw)
            overwrite_line(file_path, 37, time_boot_ms)
        elif msg.get_type() == 'RAW_IMU':
            # 线加速度x,y,z
            xacc = str(msg.xacc)
            yacc = str(msg.yacc)
            zacc = str(msg.zacc)
            x_y_z_acc = str(
                math.sqrt(msg.xacc * msg.xacc + msg.yacc * msg.yacc + msg.zacc * msg.zacc))
            # 角加速度x,y,z
            xgyro = str(msg.xgyro)
            ygyro = str(msg.ygyro)
            zgyro = str(msg.zgyro)
            # 磁场x, y, z
            xmag = str(msg.xmag)
            ymag = str(msg.ymag)
            zmag = str(msg.zmag)
            x_y_z_mag = str(
                math.sqrt(msg.xmag * msg.xmag + msg.ymag * msg.ymag + msg.zmag * msg.zmag))
            # 温度
            overwrite_line(file_path, 7, xacc)
            overwrite_line(file_path, 8, yacc)
            overwrite_line(file_path, 9, zacc)
            overwrite_line(file_path, 39, x_y_z_acc)
            overwrite_line(file_path, 10, xgyro)
            overwrite_line(file_path, 11, ygyro)
            overwrite_line(file_path, 12, zgyro)
            overwrite_line(file_path, 16, xmag)
            overwrite_line(file_path, 17, ymag)
            overwrite_line(file_path, 18, zmag)
            overwrite_line(file_path, 38, x_y_z_mag)

        elif msg.get_type() == 'SCALED_PRESSURE':
            # 压力
            temperature = str(msg.temperature)
            press_abs = str(msg.press_abs)
            press_diff = str(msg.press_diff)
            overwrite_line(file_path, 36, temperature)
            overwrite_line(file_path, 40, press_abs)
            overwrite_line(file_path, 19, press_diff)
        elif msg.get_type() == 'RC_CHANNELS':
            # 手柄
            chan1_raw = str(msg.chan1_raw)
            chan2_raw = str(msg.chan2_raw)
            chan3_raw = str(msg.chan3_raw)
            chan4_raw = str(msg.chan4_raw)
            chan5_raw = str(msg.chan5_raw)
            chan6_raw = str(msg.chan6_raw)
            chan7_raw = str(msg.chan7_raw)
            chan8_raw = str(msg.chan8_raw)
            chan9_raw = str(msg.chan9_raw)
            chan10_raw = str(msg.chan10_raw)
            chan11_raw = str(msg.chan11_raw)

            overwrite_line(file_path, 20, chan1_raw)
            overwrite_line(file_path, 21, chan2_raw)
            overwrite_line(file_path, 22, chan3_raw)
            overwrite_line(file_path, 23, chan4_raw)
            overwrite_line(file_path, 24, chan5_raw)
            overwrite_line(file_path, 25, chan6_raw)
            overwrite_line(file_path, 26, chan7_raw)
            overwrite_line(file_path, 27, chan8_raw)
            overwrite_line(file_path, 28, chan9_raw)
            overwrite_line(file_path, 29, chan10_raw)
            overwrite_line(file_path, 30, chan11_raw)
            overwrite_line(file_path, 31, "0.984657")
            overwrite_line(file_path, 32, "113.58762")
            overwrite_line(file_path, 33, "37.86236")
        elif msg.get_type() == '':
            # 湿度
            # file.write('press_abs :' + '\t' + str(msg.press_abs))
            # file.write('\n')
            pass
        # elif msg.get_type() == 'GLOBAL_POSITION_INT':
        #     # 经度 纬度 时间
        #     lat = 'lat:' + '\t' + str(msg.lat)
        #     lon = 'lon:' + '\t' + str(msg.lon)
        #     overwrite_line(file_path, 32, "113.58762")
        #     overwrite_line(file_path, 33, "37.86236")

        elif msg.get_type() == 'AHRS2':
            # 深度
            altitude = str(msg.altitude)
            overwrite_line(file_path, 34, altitude)
        elif msg.get_type() == 'SYS_STATUS':
            # 电池电压，当前电流
            voltage_battery = str(msg.voltage_battery)
            current_battery = str(msg.current_battery)
            overwrite_line(file_path, 41, voltage_battery)
            overwrite_line(file_path, 42, current_battery)

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK




# 解锁
os.environ["MAVLINK20"] = '1'
from pymavlink import mavutil




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
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        *rc_channel_values)  # RC channel list, in microseconds.


"""
1、巡线  yaw  for
2、过门  up   rl   for
3、抓球and放球  up   rl 
4、撞球  up  rl   for 
2 3 4设置为一个对准
"""


def static():
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(3, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)


def move_patrol(pwm1, pwm2, pwm3):
    set_rc_channel_pwm(3, pwm1)  # 上浮下潜
    set_rc_channel_pwm(4, pwm2)  # yaw
    set_rc_channel_pwm(5, pwm3)  # forward
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(6, 1500)


def throttle(pwm):
    # 只进行上浮下潜
    set_rc_channel_pwm(3, pwm)  # 上浮下潜
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)
    set_rc_channel_pwm(9, 1300)
    set_rc_channel_pwm(10, 1300)


def yaw(pwm):
    # 只进行偏航 大于1500右转 小于1500左转
    set_rc_channel_pwm(4, pwm)
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(3, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)
    set_rc_channel_pwm(9, 1700)
    set_rc_channel_pwm(10, 1700)


def aim(pwm1, pwm2, pwm3, pwm4):
    set_rc_channel_pwm(3, pwm1)  # 上浮下潜
    set_rc_channel_pwm(4, pwm2)  # 左拐右拐
    set_rc_channel_pwm(5, pwm3)  # 前进后退
    set_rc_channel_pwm(6, pwm4)  # 左右平移
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(9, 1100)
    set_rc_channel_pwm(10, 1100)


def lights_1_level(pwm):
    set_rc_channel_pwm(9, pwm)


def lights_2_level(pwm):
    set_rc_channel_pwm(10, pwm)


def forward(pwm4, pwm5):
    # 前后
    set_rc_channel_pwm(3, pwm4)  # 上浮下潜
    set_rc_channel_pwm(5, pwm5)  # 前进后退
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(6, 1500)


def init():
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(3, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)
    set_rc_channel_pwm(7, 1500)
    set_rc_channel_pwm(8, 1500)
    set_rc_channel_pwm(9, 1100)
    set_rc_channel_pwm(10, 1100)
    set_rc_channel_pwm(11, 1500)
    print('init')
    return True


def init_force():
    while (True):
        set_rc_channel_pwm(1, 1500)
        set_rc_channel_pwm(2, 1500)
        set_rc_channel_pwm(3, 1500)
        set_rc_channel_pwm(4, 1500)
        set_rc_channel_pwm(5, 1500)
        set_rc_channel_pwm(6, 1500)
        set_rc_channel_pwm(7, 1500)
        set_rc_channel_pwm(8, 1500)
        set_rc_channel_pwm(9, 1100)
        set_rc_channel_pwm(10, 1100)
        set_rc_channel_pwm(11, 1500)
        print('init_force')

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



if __name__ == '__main__':
    # make_print_to_file(path='/')
    print("go!")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
    print(81432)
    master.wait_heartbeat()
    print(12345)
    disarm()
    init()
    arm()
    print('Initialising stream...')
    aim(1500, 1500, 1500, 1500)
    print(999)
    read_target_param(True)
    state = 'go_forward'  # the state of rob
    number = 0  # the number of scratch
    depth_now = 0
    depth_last = -99999999
    time_n = 0  # 'find nothing' time
    #net = yolov5_lite("v5lite_s/best_g.onnx",  # Class yolov5_lite
    #                   "v5lite_s/urpc.names", 0.01, 0.01)
    model = Yolov5ONNX('v5lite_s/yolov5s_urpc_last.onnx')                                      # Class yolov5
    seefood_x1 = 0  # x in camera1
    seefood_y1 = 0  # y in camera1
    seefood_x1_last = 0
    seefood_y1_last = 0
    sth_lost = False  # flag judge object lost
    waited = 0  # wait for camera
    video = Video()  # camera on rob
    # video2 = cv2.VideoCapture(0)                                                     # camera on computer/head
    count = 0  # count in video
    rounds2 = 0  # temp count for run time
    rounds3 = 0  # count for depth
    rounds5 = 0  # count to prevent recognition flickering
    on_the_ground = False  # flag on the ground
    grip = False  # flag grip
    sth1 = False  # flag Object in camera1




    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting streaming - press "q" to quit.')

    # init the gripper
    set_servo_pwm(1, 1980)
    time.sleep(1.5)

    aim(1500, 1500, 1500, 1500)
    set_servo_pwm(2, 1582)
    time.sleep(1.5)

    aim(1500, 1500, 1500, 1500)
    set_servo_pwm(1, 1310)
    time.sleep(1.5)

    aim(1500, 1500, 1500, 1500)
    set_servo_pwm(2, 1365)
    time.sleep(1.5)
    aim(1500, 1500, 1500, 1500)

    if state == 'go_forward':
        print(state)
        while True:
            if video.frame_available():
                count = (count + 1) % 1
                if count == 0:
                    frame = video.frame()
                    frame1 = frame
                    cv2.imshow('frame', frame1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            depth_now = read_depth_continual(True)
            pwm1 = int(yaw_pid_z.calculate(80, depth_now))
            aim(pwm1, 1500, 1600, 1500)
            time.sleep(0.01)
            rounds3 += 1
            if rounds3 > 1500:
                rounds3 = 0
                break
        state = 'diving_fast'

    if state == 'diving_fast':
        print(state)
        while True:
            if video.frame_available():
                count = (count + 1) % 1
                if count == 0:
                    frame = video.frame()
                    frame1 = frame
                    cv2.imshow('frame', frame1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            aim(1200, 1500, 1500, 1500)
            time.sleep(0.01)
            if read_depth_continual(True) > 300:
                break  #dive depth
        state = 'search'

    if state == 'search':
        print(state)
        while True:
            if video.frame_available():
                count = (count + 1) % 1
                if count == 0:
                # Only retrieve and display a frame if it's new
                    frame = video.frame()
                    frame1 = frame
                    cv2.imshow('frame', frame)
                    frame1 = cv2.resize(frame1, (640, 640))

                    # yolov5
                    output, or_img = model.inference(frame1)
                    outbox = filter_box(output, 0.7, 0.7)
                    if outbox.any():
                        frame1, seefood_x1, seefood_y1, sth1 = draw(or_img, outbox, 335, 575)

                    # yolov5_lite
                    #frame1, seefood_x1, seefood_y1, sth1 = net.detect(frame, 330, 575)

                    seefood_x1 = float(seefood_x1/640)
                    seefood_y1 = float(1 - float(seefood_y1/640))
                    print(seefood_x1, seefood_y1)
                    cv2.imshow('frame_search', frame1)
            # key 'q' to stop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # xy judge grip
            if seefood_x1 > 0.47 and seefood_x1 < 0.53 and seefood_y1 > 0.08 and seefood_y1 < 0.12:        #!!!
                grip = True
            else:
                grip = False
                # near += 1
                # print("near")
                # print(near)

            #     if near > 50:
            #         grip = True
            #     else:
            #         grip = False
            # else:
            #     grip = False

            # depth judge on_the_ground
            depth_last = depth_now
            depth_now = read_depth_continual(True)
            if depth_now - depth_last < 0.3 and depth_now - depth_last > -0.3:
                rounds2 += 1
            if depth_now - depth_last >= 0.3 or depth_now > depth_last:
                if rounds2 > 3:
                    rounds2 -= 2
            if depth_now < 0:
                rounds2 = 0
            if rounds2 > 40:
                on_the_ground = True
            else:
                on_the_ground = False

            print(sth1, on_the_ground, grip)
            print(rounds2)

            # sth False
            if sth1 == False:
                if not on_the_ground:
                    pwm1 = 1405
                    pwm2 = 1450
                    aim(pwm1, pwm2, 1500, 1500)                            # down and rotate anticlockwise
                if on_the_ground:
                    rounds5 += 1
                    if (time_n + 1) % 2 == 0 and rounds5 > 6:
                        rounds5 = 2
                        print('change position left')
                        while True:
                            if video.frame_available():
                                count = (count + 1) % 1
                                if count == 0:
                                    frame = video.frame()
                                    frame1 = frame
                                    cv2.imshow('frame', frame1)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                            rounds3 += 1

                            pwm2 = 1550
                            pwm3 = 1575
                            aim(1500, pwm2, 1500, 1500)
                            time.sleep(0.01)
                            if rounds3 == 201:
                                rounds3 = 0
                                break
                        while True:
                            if video.frame_available():
                                count = (count + 1) % 1
                                if count == 0:
                                    frame = video.frame()
                                    frame1 = frame
                                    cv2.imshow('frame', frame1)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                            rounds3 += 1
                            pwm3 = 1575
                            aim(1400, 1500, pwm3, 1500)
                            time.sleep(0.01)
                            if rounds3 == 201:
                                rounds3 = 0
                                time_n = time_n + 1
                                break
                        rounds2 = 10
                    if (time_n + 1) % 2 == 1 and rounds5 > 6:
                        rounds5 = 2
                        print('change position right')
                        while True:
                            if video.frame_available():
                                count = (count + 1) % 1
                                if count == 0:
                                    frame = video.frame()
                                    frame1 = frame
                                    cv2.imshow('frame', frame1)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                            rounds3 += 1
                            pwm3 = 1575
                            pwm2 = 1450
                            aim(1500, pwm2, pwm3, 1500)
                            time.sleep(0.01)
                            if rounds3 == 201:
                                rounds3 = 0
                                break
                        while True:
                            if video.frame_available():
                                count = (count + 1) % 1
                                if count == 0:
                                    frame = video.frame()
                                    frame1 = frame
                                    cv2.imshow('frame', frame1)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                            rounds3 += 1
                            pwm3 = 1575
                            aim(1400, 1500, pwm3, 1500)
                            time.sleep(0.01)
                            if rounds3 == 201:
                                rounds3 = 0
                                time_n = time_n + 1
                                break
                        rounds2 = 10
                    if time_n >= 7 and rounds5 > 6:
                        rounds5 = 0
                        print('change ')
                        while True:
                            if video.frame_available():
                                count = (count + 1) % 1
                                if count == 0:
                                    frame = video.frame()
                                    frame1 = frame
                                    cv2.imshow('frame', frame1)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                            rounds3 += 1
                            pwm2 = 1440
                            aim(1500, pwm2, 1500, 1500)
                            time.sleep(0.01)
                            if rounds3 == 201:
                                rounds3 = 0
                                break
                        while True:
                            if video.frame_available():
                                count = (count + 1) % 1
                                if count == 0:
                                    frame = video.frame()
                                    frame1 = frame
                                    cv2.imshow('frame', frame1)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                            rounds3 += 1
                            pwm3 = 1570
                            aim(1400, 1500, pwm3, 1500)
                            time.sleep(0.01)
                            if rounds3 == 301:
                                rounds3 = 0
                                time_n = 0
                                break

            # sth True
            if sth1 == True:
                rounds5 = 0
                if not on_the_ground and not grip:
                    pwm1 = 1405
                    pwm2 = int(yaw_pid_x.calculate(0.5, seefood_x1))

                    print("x")
                    print(pwm2)
                    pwm3 = int(yaw_pid_y.calculate(0.1, seefood_y1))
                    print("y")
                    print(pwm3)
                    aim(pwm1, pwm2, pwm3, 1500)  # move to target
                if on_the_ground and not grip:
                    pwm1 = 1405
                    pwm2 = int(yaw_pid_x.calculate(0.5, seefood_x1))

                    rounds2 -= 1
                    print("x")
                    print(pwm2)
                    pwm3 = int(yaw_pid_y.calculate(0.1, seefood_y1))
                    print("y")
                    print(pwm3)

                    print(seefood_x1, seefood_y1)
                    aim(pwm1, pwm2, pwm3, 1500)  # when on the ground but not grip change posture
                if grip:                                  # grip
                    # print(depth_now)
                    print('grip!')
                    pwm1 = 1350

                    aim(pwm1, 1500, 1500, 1500)
                    # 机械臂放下
                    set_servo_pwm(1, 1980)
                    time.sleep(1.5)
                    pwm1 = 1325

                    aim(pwm1, 1500, 1500, 1500)
                    # 机械爪闭合
                    set_servo_pwm(2, 1582)
                    time.sleep(0.5)
                    pwm1 = 1325

                    aim(pwm1, 1500, 1570, 1500)
                    # 机械臂（通道三）收回
                    set_servo_pwm(1, 1310)
                    time.sleep(1.5)
                    pwm1 = 1325

                    aim(pwm1, 1500, 1500, 1500)
                    # 机械爪张开
                    set_servo_pwm(2, 1365)
                    time.sleep(0.5)
                    pwm1 = 1325

                    aim(pwm1, 1500, 1500, 1500)
                    number = number + 1
                    print("number")
                    print(number)
                    while True:                                # up after grip
                        if video.frame_available():
                            count = (count + 1) % 1
                            if count == 0:
                                frame = video.frame()
                                frame1 = frame
                                cv2.imshow('frame', frame1)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                        rounds3 += 1
                        pwm1 = 1500
                        aim(pwm1, 1500, 1500, 1500)
                        time.sleep(0.1)
                        if rounds3 == 21:
                            rounds3 = 0
                            rounds2 = 30
                            break
        cv2.destroyAllWindows()
