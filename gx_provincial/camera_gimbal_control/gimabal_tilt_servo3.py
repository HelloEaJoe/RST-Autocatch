import RPi.GPIO as GPIO
import time
import sys
import tty
import termios

# 垂直舵机引脚（BCM编号）
TILT_PIN = 2

# 垂直角度范围（根据云台机械限位调整，避免卡住）
MIN_TILT = 30   # 最小角度（例如向上30°，防止镜头朝天）
MAX_TILT = 150  # 最大角度（例如向下150°，防止镜头朝地）

# 初始角度（中间位置）
current_tilt = 90

# 角度转PWM占空比（SG90舵机校准，不同舵机可能需调整）
def angle_to_duty_cycle(angle):
    # 0°对应2.5%，180°对应12.5%（线性映射）
    return 2.5 + (angle / 180.0) * 10.0

# 初始化舵机
def init_servo():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(TILT_PIN, GPIO.OUT)
    
    # 初始化PWM，频率50Hz（舵机标准频率）
    tilt_pwm = GPIO.PWM(TILT_PIN, 50)
    tilt_pwm.start(angle_to_duty_cycle(current_tilt))
    return tilt_pwm

# 调整垂直角度（带限位保护）
def set_tilt_angle(pwm, current_angle, target_angle):
    if MIN_TILT <= target_angle <= MAX_TILT:
        pwm.ChangeDutyCycle(angle_to_duty_cycle(target_angle))
        return target_angle
    return current_angle  # 超出范围则保持当前角度

# 读取键盘输入（无阻塞）
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# 主函数
def main():
    tilt_pwm = init_servo()
    while 1:
        print("请输入角度")
        i=int(input())
        set_tilt_angle(tilt_pwm,i,i+1)

if __name__ == "__main__":
    main()