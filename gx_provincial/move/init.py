from move.move import *
def init():
    set_rc_channel_pwm(1,1500)
    set_rc_channel_pwm(2,1500)
    set_rc_channel_pwm(3,1500)
    set_rc_channel_pwm(4,1500)
    set_rc_channel_pwm(5,1500)
    set_rc_channel_pwm(6,1500)
    set_rc_channel_pwm(7,1500)
    set_rc_channel_pwm(8,1500)
    set_rc_channel_pwm(9,1100)
    #set_rc_channel_pwm(10,1100)
    #set_rc_channel_pwm(11,1500)
    print('init')
    return True

def init_force():
    while(True):
        set_rc_channel_pwm(1,1500)
        set_rc_channel_pwm(2,1500)
        set_rc_channel_pwm(3,1500)
        set_rc_channel_pwm(4,1500)
        set_rc_channel_pwm(5,1500)
        set_rc_channel_pwm(6,1500)
        set_rc_channel_pwm(7,1500)
        set_rc_channel_pwm(8,1500)
        set_rc_channel_pwm(9,1100)
        #set_rc_channel_pwm(10,1100)
        #set_rc_channel_pwm(11,1500)
        print('init_force')