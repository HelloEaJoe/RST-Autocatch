import time
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(16, GPIO.OUT)
    GPIO.setup(17, GPIO.OUT)
    GPIO.setup(6, GPIO.OUT)
except (ImportError, RuntimeError):
    # 非树莓派环境或未以root运行时，跳过GPIO设置
    class MockGPIO:
        HIGH = 1
        LOW = 0
        BOARD = None
        OUT = None
        def setmode(self, *args, **kwargs): pass
        def setup(self, *args, **kwargs): pass
        def output(self, *args, **kwargs): pass
    GPIO = MockGPIO()

GPIO.setup(16, GPIO.HIGH)
GPIO.setup(17, GPIO.HIGH)
GPIO.setup(6, GPIO.HIGH)

def red_on():
    GPIO.output(16, GPIO.HIGH)  # RED IS 16
  
def red_off():
    GPIO.output(16, GPIO.LOW)

def green_on():
    GPIO.output(17, GPIO.HIGH)  # GREEN IS 17
    
def green_off():
    GPIO.output(17, GPIO.LOW)
    
def blue_on():
    GPIO.output(6, GPIO.HIGH)  # BLUE IS 6

def blue_off():
    GPIO.output(6, GPIO.LOW)
    
def purple_on():
    GPIO.output(16, GPIO.HIGH)  # RED IS 16
    GPIO.output(17, GPIO.HIGH)  # G IS 17
    
def purple_off():
    GPIO.output(16, GPIO.LOW)
    GPIO.output(17, GPIO.LOW)

def red_blink():
    red_on()
    time.sleep(1)
    red_off()
    time.sleep(1)
    red_on()
    time.sleep(1) 
    red_off()
    time.sleep(1)
    red_on()
    time.sleep(1)
    red_off()
    time.sleep(1)

def green_blink():
    green_on()
    time.sleep(1)
    green_off()
    time.sleep(1)
    green_on()
    time.sleep(1) 
    green_off()
    time.sleep(1)
    green_on()
    time.sleep(1)
    green_off()
    time.sleep(1)

def blue_blink():
    blue_on()
    time.sleep(1)
    blue_off()
    time.sleep(1)
    blue_on()
    time.sleep(1) 
    blue_off()
    time.sleep(1)
    blue_on()
    time.sleep(1)
    blue_off()
    time.sleep(1)

def purple_blink():
    purple_on()
    time.sleep(0.05)
    purple_off()
    time.sleep(0.05)
    purple_on()
    time.sleep(0.05) 
    purple_off()
    time.sleep(0.05)
    purple_on()
    time.sleep(0.05)
    purple_off()
    time.sleep(0.05)

if __name__=='__main__':
    # while True:
    red_blink()
    green_blink()
    blue_blink()
    purple_blink()
        # purple_on()
        # time.sleep(0.05)
        # purple_off()
        # time.sleep(0.05)
        #GPIO.output(16,GPIO.HIGH)#RED IS 16    
        #     time.sleep(0.05)
        #GPIO.output(16,GPIO.LOW)
        #time.sleep(0.05)
        #GPIO.output(17,GPIO.HIGH)#GREEN IS 17    
        #     time.sleep(0.05)
        #GPIO.output(17,GPIO.LOW)
        #     time.sleep(0.05)
        #GPIO.output(6,GPIO.HIGH)#BLUE IS 6    
        #     time.sleep(0.05)
        #GPIO.output(6,GPIO.LOW)
        #     time.sleep(0.05)

        
        
        
