# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import maestro

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

#primary windows
cv.namedWindow("Video")
cv.namedWindow("HSV")
cv.namedWindow("BW")

#initialize variables so we can use 'em
hsv = 0
minHSV = np.array([0, 0, 0])
maxHSV = np.array([0, 0, 0])
res = [0, 0, 0]
kernel = np.ones((5,5), np.uint8)


#functions for debugging sliders
def change_hue(value):
    print("change hue")

def change_sat(value):
    print("change saturation")

def change_val(value):
    print("change value")

#set new pixel to track when clicked (only for HSV window)
def mouseCall(evt, x, y, flags, pic):
    if evt==cv.EVENT_LBUTTONDOWN:
        global res
        res = hsv[y][x]
        print(str(x) + ", " + str(y))
        print(res)

#set scalar values for tracking every loop
def setScalars(h, s, v):
    global res, minHSV, maxHSV
    minHSV = np.array([res[0] - h, res[1] - s, res[2] - v])
    maxHSV = np.array([res[0] + h, res[1] + s, res[2] + v])

#create sliders
cv.createTrackbar("Hue Tol.", "HSV", 0, 255, change_hue)
cv.createTrackbar("Sat Tol.", "HSV", 0, 255, change_sat)
cv.createTrackbar("Val Tol.", "HSV", 0, 255, change_val)
cv.setMouseCallback("HSV", mouseCall, hsv)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    img = frame.array
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV) #convert to HSV
    bw = cv.inRange(hsv, minHSV, maxHSV)    #black and white image to track stuff
    img_erosion = cv.erode(bw, kernel, iterations=2)    #erode white
    img_dilation = cv.dilate(img_erosion, kernel, iterations=2) #dilate black

    #create images in windows
    cv.imshow("Video", img)
    cv.imshow("HSV", hsv)
    cv.imshow("BW", bw)
    cv.imshow('Erosion', img_erosion)
    cv.imshow('Dilation', img_dilation)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
            break

    #get current positions of three trackbars
    h = cv.getTrackbarPos('Hue Tol.', 'HSV')
    s = cv.getTrackbarPos('Sat Tol.', 'HSV')
    v = cv.getTrackbarPos('Val Tol.', 'HSV')

    #update tracking scalars
    setScalars(h, s, v)

cv.destroyAllWindows()