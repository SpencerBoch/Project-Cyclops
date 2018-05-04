import cv2
import numpy as np
import time
import threading
import RPi.GPIO as GPIO
import picamera

import setup
from setup import setupPins

#finds new color
def newColor():
    print ("finding new color!")
    #lock or block
    setup.mutex.acquire()

    newImg = np.empty((480*640*3,), dtype = np.uint8) #openCV uses numpy arrays as images

    #take image
    setup.camera.capture(newImg, 'bgr')
    newImg = newImg.reshape((480, 640, 3))

    #convert to HSV
    newImgHSV = cv2.cvtColor(newImg, cv2.COLOR_BGR2HSV)#convert image to HSV
    
    #get values of HSV image from center
    pixel = newImgHSV[240, 320]

    y = pixel[0] - 40
    x = pixel[0] + 50

    #make sure X and Y is in bounds (0 - 255)
    if (x > 255):
        x = 255

    if (y < 0):
        y = 0
        
            
    setup.lBound = np.array([int(y), 50, 50])
    setup.uBound = np.array([int(x), 255, 255])

    #release
    setup.mutex.release()
    

def singlePoint():
    print ("default mode")
    
    #setup pins for this thread
    setupPins()
    
    #turn on middle led
    GPIO.output(18, GPIO.HIGH)

    #move servo to 90* position
    setup.servoMutex.acquire()
    
    setup.servo.ChangeFrequency(100)
    setup.servo.ChangeDutyCycle(12.5)
    setup.servo.ChangeFrequency(5)

    setup.servoMutex.release()
    
    while(True):
        time.sleep(.5)
        if (setup.state != 2):
            print "exiting single fire"

            #cleanup pins before exit
            GPIO.cleanup()
            setup.servo.stop()
            return

     
#seek mode, seraches for colored object 
def seek():
    prevY = -3
    ledOff = 20

    #setup pins in this thread
    setupPins()

    #loop to keep taking pics
    while (True):

        #lock
        setup.mutex.acquire()
        
        #keep taking picktures until cancelled
        img = np.empty((480*640*3,), dtype = np.uint8) #openCV uses numpy arrays as images
        setup.camera.capture(img, 'bgr')

        img = img.reshape((480, 640, 3))

        #release lock
        setup.mutex.release()
        
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#convert image to HSV

        #mask to cover up some of the holes in HSV image
        mask = cv2.inRange(imgHSV, setup.lBound, setup.uBound)
        kernelOpen = np.ones((5,5))
        kernelClose = np.ones((20,20))

        #mask to cover up MORE holes in image
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE,kernelClose)
        maskFinal = maskClose

        #establish contours to draw around green object
        conts, h = cv2.findContours(maskFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img, conts, -1, (255,0,0),3) # draw contours

        #initalize values initially to 0
        largestW = 0
        largestH = 0
        largestX = 0
        largestY = 0

        #for each contour, find biggest box and save it to largestC
        for i in range(len(conts)):
            x,y,w,h = cv2.boundingRect(conts[i])
            cv2.rectangle(img,(x,y),(x+w,y+h), (0,0,255), 2)
            cv2.cv.PutText(cv2.cv.fromarray(img), str(i+1),(x,y+h),setup.font,(0,255,255))    

            if((w*h) >= largestW*largestH):
                largestW = w
                largestH = h
                largestX = x
                largestY = y

        largestX = largestX + (largestW/2)
        largestY = largestY + (largestH/2)            

        if (largestW*largestH < 4000):
            largestX = 0
            largestY = 0
        
        #set all LED pins to LOW/OFF
        GPIO.output(ledOff, GPIO.LOW)

        #if statements to determine which LED to turn on
        if (largestX < 1):
            print "nothing on screen, print default led"
            GPIO.output(18, GPIO.HIGH)
            ledOff = 18
            
        elif(largestX <= 91 and largestX > 0):
            print("LED 1")
            GPIO.output(20, GPIO.HIGH)
            ledOff = 20

        elif(largestX >= 92 and largestX <= 183):
            print("LED 2")
            GPIO.output(16, GPIO.HIGH)
            ledOff = 16

        elif(largestX >= 184 and largestX <= 275):
            print("LED 3")
            GPIO.output(12, GPIO.HIGH)
            ledOff = 12

        elif(largestX >= 276 and largestX <= 367):
            print("LED 4")
            GPIO.output(18, GPIO.HIGH)
            ledOff = 18

        elif(largestX >= 368 and largestX <= 459):
            print("LED 5")
            GPIO.output(25, GPIO.HIGH)
            ledOff = 25

        elif(largestX >= 460 and largestX <= 550):
            print("LED 6")
            GPIO.output(23, GPIO.HIGH)
            ledOff = 23

        elif(largestX >= 551 and largestX < 640):
            print("LED 7")
            GPIO.output(7, GPIO.HIGH)
            ledOff = 7

        i = .5 #determines time to sleep, .5 seconds, allows servo to get to position first, then change frequency to reduce jitter
        j = 5 #frequency to lower to to reduce jitter


        setup.servoMutex.acquire()
        
        #if statements to determine position of servo
        if (largestY < 1):
            if(prevY != 2):
                print "nothing on!, default servo"
                setup.servo.ChangeFrequency(100)
                time.sleep(i)
                setup.servo.ChangeDutyCycle(12.5)
                time.sleep(i)
                setup.servo.ChangeFrequency(j)
                prevY = 2
                
        elif(largestY <= 160 and largestY > 0):
            #if servo already in position 1, don't bother changing
            if(prevY != 1):
                print "servo 1"
                setup.servo.ChangeFrequency(100)
                time.sleep(i)
                setup.servo.ChangeDutyCycle(8)
                time.sleep(i)
                setup.servo.ChangeFrequency(j)
                prevY = 1
                        
        elif(largestY >= 161 and largestY <= 320):
            if(prevY != 2):
                print "servo 2"
                setup.servo.ChangeFrequency(100)
                time.sleep(i)
                setup.servo.ChangeDutyCycle(12.5)
                time.sleep(i)
                setup.servo.ChangeFrequency(j)
                prevY = 2

        elif(largestY >= 321):
            if(prevY != 3):
                print "servo 3"
                setup.servo.ChangeFrequency(100)
                time.sleep(i)
                setup.servo.ChangeDutyCycle(17)
                time.sleep(i)
                setup.servo.ChangeFrequency(j)
                prevY = 3

        setup.servoMutex.release()
        
        if (setup.state != 1):
            #turn off recent led
            print ("EXITING")
            GPIO.output(ledOff, GPIO.LOW)

            #cleanup this thread's pins before exiting.
            GPIO.cleanup()
            setup.servo.stop()
            return
                    
def start():
    print ("in startup!")
    if (setup.state == 1):
        print("entering seek mode")
        threading.Thread(target=seek).start()
        





    


