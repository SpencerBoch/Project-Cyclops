import time
import RPi.GPIO as GPIO
import threading
import setup
from modes import *

#button interrupt, changes states
def stateChange(channel):
    print "button pressed!"
    setup.state = setup.state + 1

    print "state = "
    print setup.state
    if (setup.state == 3):
        setup.state = 1

    print (setup.state)
    
    if(setup.state == 1):
        threading.Thread(target=seek).start()
        
    elif(setup.state == 2):
        threading.Thread(target=singlePoint).start()

    time.sleep(1)
    if(GPIO.input(11)):
        print "button held!"
        threading.Thread(target=newColor).start()
                    
try:
    #setup button
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(11, GPIO.OUT)
    GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    #setup event
    GPIO.add_event_detect(11, GPIO.RISING, callback=stateChange, bouncetime = 300)

    #run setup code
    setup.setupCam()

    #start program
    start()

    #keep main program running so threads can be issued
    while (True):
        time.sleep(50)
        print "standby"

        
    
#if terminated, run this code before exit.
except KeyboardInterrupt:
    setup.servo.stop()

GPIO.cleanup()

            






    


