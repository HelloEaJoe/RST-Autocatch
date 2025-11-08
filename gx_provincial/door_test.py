import time
from pymavlink import mavutil
from move.arm_disarm import arm,disarm
from move.init import *
from move.move import *
from pid_catalog_first.set_deepth import*
from pid_catalog_first.sheer_pid import *


def pwm_with_delay(pwm1,pwm2,pwm3, delay):
    start_time = time.perf_counter()
    while True:
        current_time = time.perf_counter()
        elapsed_time = current_time - start_time
        if elapsed_time >= delay:
            aim(pwm1,pwm2,pwm3)
            break







if __name__=='__main__':

    print("GO!")
    master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
    master.wait_heartbeat()
    disarm()
    init()
    arm()
    state = 'start'
    global pwm1, pwm2, pwm3
    pwm1, pwm2, pwm3 = 1500, 1500, 1500
    line_x = 10
    line_y = 10
    ball_x = 5
    ball_y = 5
    door_centre_x = 20
    door_centre_y = 20
    round = 0
    leader_line_last = False
    leader_line_now = False

    door_last = False
    door_now = False
    ball_last = False
    ball_now = False
    print(state)
    aim(1500,1500,1500)

    while True:
        print(state)

        if state == 'start':
            #pwm1 = 1600
            pwm_with_delay(1600, 1500, 1500, 0)
            #在第二秒停止下沉
            pwm_with_delay(1500, 1500, 1500, 2)
            aim(pwm1, pwm2, pwm3)
            state = 'leader_line'


        if state == 'leader_line_first':
            print(state)
            if leader_line_last is None and leader_line_now is None:
                pwm2 = 1450

                pwm1 = 1525          #模拟定深
                aim(pwm1, pwm2, 1500)

            if leader_line_now is True:
                pwm2 = int(yaw_pid_leaderline_x.calculate(0.5,line_x,1495))
                pwm1 = int(yaw_pid_leaderline_y.calculate(0.5,line_y,1495))
                aim(pwm1, pwm2, 1600)

            if leader_line_last is True and leader_line_now is None:
                state = 'balloon'
                print('line to balloon')

        if state == 'balloon':
            print(state)
            if ball_last is False and ball_now is False:
                aim(1500, 1550, 1500)

            if ball_now is True:
                pwm2 = int(yaw_pid_ball_x.calculate(0.5, ball_x, 1495))
                pwm1 = int(yaw_pid_ball_y.calculate(0.5, ball_y, 1495))
                aim(pwm1, pwm2, 1600)

            if ball_last is True and ball_now is False:
                #pwm3 = 1600
                pwm_with_delay(1500, 1500, 1600, 0)
                #前进2秒
                pwm_with_delay(1500, 1500, 1500, 2)

                #碰撞完成退回原位
                pwm_with_delay(1500, 1500, 1400, 0)
                pwm_with_delay(1500, 1500, 1500, 2)

                #调个头
                pwm_with_delay(1500, 1550, 1500, 0)
                pwm_with_delay(1500, 1500, 1500, 2)

                aim(pwm1, pwm2, pwm3)

                state = 'leader_line_second'

        if state == 'leader_line_second':
            print(state)
            print(round)
            round += 1
            if leader_line_last is None and leader_line_now is None:
                pwm2 = 1450

                pwm1 = 1525  # 模拟定深
                aim(pwm1, pwm2, 1500)

            if leader_line_now is True:
                pwm2 = int(yaw_pid_leaderline_x.calculate(0.5, line_x, 1495))
                pwm1 = int(yaw_pid_leaderline_y.calculate(0.5, line_y, 1495))
                aim(pwm1, pwm2, 1600)

            if leader_line_last is True and leader_line_now is None:
                state = 'Through the door'
                print('line to door')


        if state == 'Through the door':
            print(state)
            if door_last is False and door_now is False:
                aim(1500, 1550, 1500)

            if ball_now is True:
                pwm2 = int(yaw_pid_door_x.calculate(0.5, door_centre_x, 1495))
                pwm1 = int(yaw_pid_door_y.calculate(0.5, door_centre_y, 1495))
                aim(pwm1, pwm2, 1600)

            if ball_last is True and ball_now is False:
                # pwm3 = 1600
                pwm_with_delay(1500, 1500, 1600, 0)
                # 前进2秒
                pwm_with_delay(1500, 1500, 1500, 2)

                state = 'leader_line_second'





















