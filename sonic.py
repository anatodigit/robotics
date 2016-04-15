import RPi.GPIO as gpio
import time

def distance():
    sonicTrigger1 = 12
    sonicEcho1 = 16
    print "top of file"
    
    try:
        print "top of try"
        gpio.setmode(gpio.BOARD)
        gpio.setup(sonicTrigger1, gpio.OUT)
        gpio.setup(sonicEcho1, gpio.IN)
        print "end setup"
        gpio.output(sonicTrigger1, False)
        print "trigger set to false"
        # Allow module to settle
        time.sleep(0.3)
        print "after sleep"
        # Send 10us pulse to trigger
        GPIO.output(sonicTrigger1, True)
        time.sleep(0.00001)
        GPIO.output(sonicTrigger1, False)
        print "end of trigger pulse"
        start = time.time()
        while GPIO.input(sonicEcho1)==0:
            start = time.time()

        while GPIO.input(sonicEcho1)==1:
            stop = time.time()
        
        
        print "start is ", start, "  end is ",stop
        # Calculate pulse length
        elapsed = stop-start

        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s)
        distance = elapsed * 34000

        # That was the distance there and back so halve the value
        distance = distance / 2
            
        gpio.cleanup()
        return distance
    except:
        distance = 100
        gpio.cleanup()
        return distance