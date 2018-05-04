import picamera
import cv2
import time
import RPi.GPIO as GPIO
import numpy as np

from threading import Lock

def setupPins():
    #set output pins to go of GPIO number, not board
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(26, GPIO.OUT)
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(7,  GPIO.OUT)
    GPIO.setup(25, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(16, GPIO.OUT)

    GPIO.setup(13, GPIO.OUT)

    
def setupCam():
    #initial setup
    #includes initialization, framerate and res
    #settings and sets it to black and white
    #time.sleep lets camera warm up and adjust to light before use
    global camera
    global img
    global servo
    global font
    global lBound
    global uBound
    global state
    global mutex
    global servoMutex
    
    mutex = Lock()
    servoMutex = Lock()
    
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 24

    img = np.empty((480*640*3,), dtype = np.uint8)
    camera.capture(img, 'bgr')
    
    cv2.imwrite("test.jpg", img)
    font = cv2.cv.InitFont(cv2.cv.CV_FONT_HERSHEY_SIMPLEX,2,0.5,0,3,1)

    #setup LED pins to OUT
    setupPins()

    #setup servo
    servo = GPIO.PWM(13, 100)
    servo.start(12.5) #initialize at 90*

    #setup button
    #GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    state = 2 #initial state for modes
    #GPIO.add_event_detect(11, GPIO.FALLING, callback=stateChange, bouncetime = 300)

    #include bounds (in HSV) for green and a name to save
    lBound = np.array([50,50,20])
    uBound = np.array([80,255,215])

    #allow pi to warmup
    time.sleep(2)
    
    


            






    


