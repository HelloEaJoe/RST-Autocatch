import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(16,GPIO.HIGH)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(17,GPIO.HIGH)
GPIO.setup(17,GPIO.OUT)
GPIO.setup(6,GPIO.HIGH)
GPIO.setup(6,GPIO.OUT) 

def red_on():
    GPIO.output(16,GPIO.HIGH)#RED IS 16    
  
def red_off():
    GPIO.output(16,GPIO.LOW)
    
def green_on():
    GPIO.output(17,GPIO.HIGH)#GREEN IS 17
    
def green_off():
    GPIO.output(17,GPIO.LOW)
    
def blue_on():
    GPIO.output(6,GPIO.HIGH)#BLUE IS 6

def blue_off():
    GPIO.output(6,GPIO.LOW)
    
def purple_on():
    GPIO.output(16,GPIO.HIGH)#RED IS 16
    GPIO.output(17,GPIO.HIGH)#G IS 17
    
def purple_off():
    GPIO.output(16,GPIO.LOW)
    GPIO.output(17,GPIO.LOW)

def green_blink():
    red_on()
    time.sleep(0.05)
    red_off()
    time.sleep(0.05)
    red_on()
    time.sleep(0.05) 
    red_off()
    time.sleep(0.05)
    red_on()
    time.sleep(0.05)
    red_off()
    time.sleep(0.05)

def red_blink():
    green_on()
    time.sleep(0.05)
    green_off()
    time.sleep(0.05)
    green_on()
    time.sleep(0.05) 
    green_off()
    time.sleep(0.05)
    green_on()
    time.sleep(0.05)
    green_off()
    time.sleep(0.05)

def blue_blink():
    blue_on()
    time.sleep(0.05)
    blue_off()
    time.sleep(0.05)
    blue_on()
    time.sleep(0.05) 
    blue_off()
    time.sleep(0.05)
    blue_on()
    time.sleep(0.05)
    blue_off()
    time.sleep(0.05)

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

        
        
        
