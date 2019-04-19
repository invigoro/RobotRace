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

class ClientSocket(threading.Thread):
    def __init__(self, IP, PORT):
        super(ClientSocket, self).__init__()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((IP, PORT))
  
        print ('connected')
        self.alive = threading.Event()
        self.alive.set()

    def recieveData(self):
        global globalVar
        try:
            data = self.s.recv(105)
            print (data)
            globalVar = data
        except IOError as e:
            if e.errno == errno.EWOULDBLOCK:
                pass

    def sendData(self, sendingString):
        print ('sending')
        sendingString += "\n"
        self.s.send(sendingString.encode('UTF-8'))
        print ('done sending')

    def run(self):
        global globalVar
        while self.alive.isSet():
            data = self.s.recv(105)
            print (data)
            globalVar = data
            if(data == "0"):
                self.killSocket()
            
           
            
    def killSocket(self):
        self.alive.clear()
        self.s.close()
        print("Goodbye")
        exit()
            

IP = '10.200.35.21'
PORT = 5010
client = ClientSocket(IP, PORT)
##client.start()
speech = "It goes it goes it goes it goes it goes YUH"
key = None

class Control():
    def __init__(self):
        self.tango = maestro.Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.resetSearchPan = False
        self.resetSearchTilt = False
    
    def resetHead(self):
        self.headTilt = 6000
        self.headTurn = 6000
        self.tango.setTarget(HEADTURN, self.headTurn)
        self.tango.setTarget(HEADTILT, self.headTilt)
        print("RESET HEAD")

    def turnRight(self):
        self.headTurn += 500
        if(self.headTurn > 7900):
            self.headTurn = 7900
        self.tango.setTarget(HEADTURN, self.headTurn)   
        print("TURN HEAD RIGHT")
    def turnLeft(self):
        self.headTurn -= 500
        if(self.headTurn < 4100):
            self.headTurn = 4100
        self.tango.setTarget(HEADTURN, self.headTurn)
        print('TURN HEAD LEFT')        
    def tiltUp(self):
        self.headTilt += 500
        if(self.headTilt > 7900):
            self.headTilt = 7900
        self.tango.setTarget(HEADTILT, self.headTilt)
    def tiltDown(self):
        self.headTilt -= 500
        if(self.headTilt < 4100):
            self.headTilt = 4100
        self.tango.setTarget(HEADTILT, self.headTilt)
    def left(self):
        self.turn += 1000
        self.tango.setTarget(TURN, self.turn)
        print("LEFT")
        time.sleep(.25)
        self.turn -= 1000
        self.tango.setTarget(TURN, self.turn)
    def right(self):
        self.turn -= 1000
        self.tango.setTarget(TURN, self.turn)
        print("RIGHT")
        time.sleep(.25)
        self.turn += 1000
        self.tango.setTarget(TURN, self.turn)
    def forward(self):
        self.motors = 5000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.3)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
    def backward(self):
        self.motors = 7000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.3)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
    def shakeHead(self):
        self.turnRight()
        time.sleep(0.5)
        self.turnLeft()
        time.sleep(0.5)
        self.turnRight()
        time.sleep(0.5)
        self.turnLeft()
        time.sleep(0.5)
        
    def searchForFaces(self):
        headTilt = self.headTilt
        headTurn = self.headTurn
        if(headTilt >= 7900):
            self.resetSearchTilt = True
        if(headTilt <= 4100):
            self.resetSearchTilt = False

        resetPan = self.resetSearchPan
        resetTilt = self.resetSearchTilt

        if(headTurn < 7900 and resetPan is False):
            self.turnRight()
        elif(headTurn >= 7900 and resetPan is False):
            self.resetSearchPan = True
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnLeft()
        elif(headTurn > 4100 and resetPan is True):
            self.turnLeft()
        elif(headTurn <= 4100 and resetPan is True):
            self.resetSearchPan = False
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnRight()

    def alignMoveToFace(self, face):
        global faceSize, faceSizeTolerance, faceTolerance, centerx, centery
        largest = 0
        current = None
        for (x,y,w,h) in faces:
            if(w*h) > largest:
                largest = w*h
                current = [int(x+(w/2)), int(y+(h/2))]
        if(self.headTurn < 5900):    #head turned left
            if(current[0] - centerx > faceTolerance):   #human is to the right
                controller.turnRight()
            elif(current[0] - centerx < -faceTolerance): #human is even farther left
                controller.left()
            else:   #human is centered but head is turned, so we turn body in opposite directions to fix
                controller.turnRight()
                controller.left()
        elif(self.headTurn > 6100): #head turned right
            if(current[0] - centerx > faceTolerance): #human is even father right
                controller.right()
            elif(current[0] - centerx < -faceTolerance): #human is to the left
                controller.turnLeft()
            else:   #human is centered but head is turned, so we turn body in opposite directions to fix
                controller.turnLeft()
                controller.left()
        elif(current[0] - centerx > faceTolerance): #human is to the right and head not turned
            controller.right()
        elif(current[0] - centerx < -faceTolerance):    #human is to the left and head not turned
            controller.left()   
        else:
            if(current[1] - centery > faceTolerance):
                controller.tiltUp()
            elif(current[1] - centery < -faceTolerance):
                controller.tiltDown()
            else:
                if(largest > faceSize + faceSizeTolerance): #human too big
                    print("Go backward")
                    #controller.backward()
                elif(largest < faceSize - faceSizeTolerance):
                    print("Go forward")
                    #controller.forward()
                else:
                    return False
        return True

    


controller = Control()