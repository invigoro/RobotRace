# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import maestro
from time import sleep
from threading import Timer
import socket
import threading
import queue
from colordict import colorDict
from controller import controller

#for erosion/dilation
kernel = np.ones((5,5), np.uint8)

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')

faceSize = 20000
faceSizeTolerance = 5000
faceTolerance = 40

timer = None
t = 5.0
key = None

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

centerx = 640 / 2
centery = 480 / 2

degreeTot = 0
direction = 1

globalVar = ""

foundBox = False

def setKey(self):
    print('key')

def detectFaces(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    thereAreSome = False
    #print(faces)
    for (x,y,w,h) in faces:
        cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        thereAreSome = True
        #roi_gray = gray[y:y+h, x:x+w]
        #roi_color = img[y:y+h, x:x+w]
        #eyes = eye_cascade.detectMultiScale(roi_gray)
    if thereAreSome is False:
        return False
    return faces


def detectLines(self, img):
    lower_pink = np.array([100, 140, 70], dtype=np.uint8)
    upper_pink = np.array([200, 160, 255], dtype=np.uint8)

    lower_white = np.array([0, 0, 0], dtype=np.uint8)
    upper_white = np.array([0, 0, 255], dtype=np.uint8)

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, lower_pink, upper_pink)
    mask2 = cv.inRange(hsv, lower_white, upper_white)
    mask = cv.bitwise_or(mask1, mask2)
    res = cv.bitwise_and(img, img, mask=mask)


key = None

def subtract(a, b):
    if len(a) is not len(b):
        return False
    val = []
    for i in range(0, len(a)):
        val.append(a[i] - b[i])
    return np.array(val)

def add(a, b):
    if len(a) is not len(b):
        return False
    val = []
    for i in range(0, len(a)):
        val.append(a[i] + b[i])
    return np.array(val)


def lookForColor(hue, tol, img):
    areaThreshold = 100
    colorMin = subtract(hue, tol)
    colorMax = add(hue, tol)

    blur = cv.blur(img, (5,5))

    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, colorMin, colorMax)

    img_erosion = cv.erode(mask1, kernel, iterations=3)    #erode white
    img_dilation = cv.dilate(img_erosion, kernel, iterations=4) #dilate black

    contours, hierarchy = cv.findContours(img_dilation, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for i in contours:
        if cv.contourArea(i) > areaThreshold:
            return True

    cv.imshow("Original", img)
    return False
    

def moveToLine(hue, tol, img, controller): #target hue, tolerance, image to mask, and robot controller
    colorMin = subtract(hue, tol)
    colorMax = add(hue,tol)

    blur = cv.blur(img, (5,5))

    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, colorMin, colorMax)
    #res = cv.bitwise_and(img, img, mask=mask)

    img_erosion = cv.erode(mask1, kernel, iterations=3)    #erode white
    img_dilation = cv.dilate(img_erosion, kernel, iterations=4) #dilate black
    cv.imshow("Dilation", img_dilation)

    #find COG and draw it
    moments = cv.moments(img_dilation, True)


    try:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        if abs(cx - centerx) < 30:
            controller.crossLine()
            return True
            #move forward
        elif cx > centerx:
            #rotate right
            controller.right()
        else:
            controller.left()
        time.sleep(0.1)
    except: #no line detected so we'll just keep rotatin'
        cx = 0
        cy = 0
        print("No detected line")
        controller.right()
        time.sleep(0.2)


    print(str(cx) + " " + str(cy))
    cv.circle(img, (cx, cy), 8, (0,0,255), -1)
    
    cv.imshow("Original", img)
    return False


def moveToBox(hue, tol, img, controller): # target hue, tolerance, image to mask, and robot controller
    global foundBox
    colorMin = subtract(hue, tol)
    colorMax = add(hue, tol)

    blur = cv.blur(img, (5, 5))

    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, colorMin, colorMax)
    # res = cv.bitwise_and(img, img, mask=mask)

    img_erosion = cv.erode(mask1, kernel, iterations=3)  # erode white
    img_dilation = cv.dilate(img_erosion, kernel, iterations=4)  # dilate black
    cv.imshow("Dilation", img_dilation)

    # find COG and draw it
    moments = cv.moments(img_dilation, True)

    try:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        tolerance = 30
        if abs(cx - centerx) < tolerance:
            controller.forward()
            foundBox = True
            # move forward
        elif cx > centerx:
            # rotate right
            controller.right()
        else:
            controller.left()
        time.sleep(0.1)
    except:  # no line detected so we'll just keep rotatin'
        cx = 0
        cy = 0
        print("No detected line")
        if foundBox == True:
            for i in range(0, 2):
                controller.forward()
            for i in range(0, 2):
                controller.waistLeft()
            controller.handReset()
            controller.elbowUpMax()
            controller.handOpen()
            return True
        controller.right()
        time.sleep(0.2)

    print(str(cx) + " " + str(cy))
    cv.circle(img, (cx, cy), 8, (0, 0, 255), -1)

    cv.imshow("Original", img)
    return False

controller.elbowReset()
controller.tiltHeadDownMax()

time.sleep(5)#wait for motors to catch up?

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
    img = frame.array
    if(moveToLine(colorDict['orange']['val'], colorDict['orange']['tol'], img, controller)) is True:
        rawCapture.truncate(0)
        #client.sendData("Entering mining area")
        break

    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
        cv.destroyAllWindows()
        break

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
    img = frame.array
    if(moveToLine(colorDict['orange']['val'], colorDict['orange']['tol'], img, controller)) is True:
        rawCapture.truncate(0)
        #client.sendData("Entering mining area")
        break

    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
        cv.destroyAllWindows()
        break

controller.resetHead()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
    img = frame.array

    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
        cv.destroyAllWindows()
        break

    faces = detectFaces(img)
    if(faces is False):
        controller.searchForFaces()
        time.sleep(0.5)
    elif(controller.alignMoveToFace(faces) is False):
        time.sleep(1)
        break
    cv.imshow("Image", img)
    
print("end of face tracking")

controller.elbowUpMax()
controller.handReset()
controller.tiltHeadDownMax()
controller.handOpen()
controller.handOpen()
print("Please give me some ice")
#client.sendData("Please please please please please please please please please give me some pink ice. I am but a poor street urchin with no ice of my own.")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to wait for pink ice
    img = frame.array

    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
        cv.destroyAllWindows()
        break

    if lookForColor(colorDict['hotpink']['val'], colorDict['hotpink']['tol'], img):
        print("That's the right color")
        #client.sendData("Oh yeah, there's the right color. Give me that PLEASE")
        controller.nodHead()
        break


controller.resetHead()
controller.tiltHeadDownMax()
controller.resetWaist()

for i in range(0, 8):
    controller.right()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
    img = frame.array
    if(moveToBox(colorDict['hotpink']['val'], colorDict['hotpink']['tol'], img, controller)) is True:
        rawCapture.truncate(0)
        #client.sendData("Entering mining area")
        break

    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
        cv.destroyAllWindows()
        break

time.sleep(4)
controller.handReset()
controller.handClose()


cv.destroyAllWindows()
