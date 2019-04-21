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

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

degreeTot = 0
direction = 1

globalVar = ""


def setKey(g):
    global key
    key = g

def detectFaces(self, img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    #print(faces)
    for (x,y,w,h) in faces:
        cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        #roi_gray = gray[y:y+h, x:x+w]
        #roi_color = img[y:y+h, x:x+w]
        #eyes = eye_cascade.detectMultiScale(roi_gray)
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

'''whiteMin = subtract(colorDict['white']['val'], colorDict['white']['tol'])
whiteMax = add(colorDict['white']['val'], colorDict['white']['tol'])
pinkMin = subtract(colorDict['pink']['val'], colorDict['pink']['tol'])
pinkMax = add(colorDict['pink']['val'], colorDict['pink']['tol'])
orangeMin = subtract(colorDict['orange']['val'], colorDict['orange']['tol'])
orangeMax = add(colorDict['orange']['val'], colorDict['orange']['tol'])'''

lineCentered = False

def moveToLine(hue, tol, img, controller): #target hue, tolerance, image to mask, and robot controller
    global lineCentered
    colorMin = subtract(hue, tol)
    colorMax = add(hue,tol)

    blur = cv.blur(img, (5,5))

    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, colorMin, colorMax)
    #res = cv.bitwise_and(img, img, mask=mask)

    img_erosion = cv.erode(mask1, kernel, iterations=3)    #erode white
    img_dilation = cv.dilate(img_erosion, kernel, iterations=4) #dilate black

    #find COG and draw it
    moments = cv.moments(img_dilation, True)


    try:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        if abs(cx - centerx) < 32:
            lineCentered = True
            #move forward
            controller.forward()
        elif cx > centerx:
            #rotate right
            controller.right()
            lineCentered = False
        else:
            controller.left()
            lineCentered = False
        time.sleep(0.1)
    except: #no line detected so we'll just keep rotatin'
        if lineCentered is True:
            controller.crossLine()
            return True
        cx = 0
        cy = 0
        print("No detected line")
        controller.right()
        lineCentered = False
        time.sleep(0.2)


    print(str(cx) + " " + str(cy))
    cv.circle(img, (cx, cy), 8, (0,0,255), -1)
    return False

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
    img = frame.array
    if(moveToLine(colorDict['pink']['val'], colorDict['pink']['tol'], img, controller)) is True:
        rawCapture.truncate(0)
        #client.sendData("Entering mining area")
        break
    '''
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV) #convert to HSV
    white = cv.inRange(hsv, whiteMin, whiteMax)
    pink = cv.inRange(hsv, pinkMin, pinkMax)
    orange = cv.inRange(hsv, orangeMin, orangeMax)
    
    cv.imshow("Video", img)
    cv.imshow("White", white)
    cv.imshow("Pink", pink)
    cv.imshow('Orange', orange)

    print(whiteMin)
    print(whiteMax)'''

    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
        cv.destroyAllWindows()
        break

cv.destroyAllWindows()
