import cv2 as cv
import numpy as np
import maestro
import time
from time import sleep
from threading import Timer

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

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
        self.turn += 1300
        self.tango.setTarget(TURN, self.turn)
        print("LEFT")
        time.sleep(.25)
        self.turn -= 1300
        self.tango.setTarget(TURN, self.turn)
    def right(self):
        self.turn -= 1300
        self.tango.setTarget(TURN, self.turn)
        print("RIGHT")
        time.sleep(.25)
        self.turn += 1300
        self.tango.setTarget(TURN, self.turn)
    def forward(self):
        self.motors = 5000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.3)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
    def crossLine(self):
        speed = 1000
        self.motors -= speed
        self.tango.setTarget(MOTORS, self.motors)
        print("Cross Line")
        time.sleep(1)
        self.motors += speed
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
    
    def tiltHeadDownMax(self):
        self.headTilt = 1510
        self.tango.setTarget(HEADTILT, self.headTilt)


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