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
SHOULDER = 6
ELBOW = 8
HAND = 11

faceSize = 10000
faceSizeTolerance = 2500
faceTolerance = 100

centerx = 640 / 2
centery = 480 / 2

class Control():
    def __init__(self):
        self.tango = maestro.Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.shoulder = 6000
        self.elbow = 6000
        self.hand = 6000
        self.resetSearchPan = False
        self.resetSearchTilt = False
    
    def resetHead(self):
        self.headTilt = 6000
        self.headTurn = 6000
        self.tango.setTarget(HEADTURN, self.headTurn)
        self.tango.setTarget(HEADTILT, self.headTilt)
        print("RESET HEAD")

    def turnLeft(self):
        self.headTurn += 250
        if(self.headTurn > 7900):
            self.headTurn = 7900
        self.tango.setTarget(HEADTURN, self.headTurn)
        print("TURN HEAD LEFT")
    def turnRight(self):
        self.headTurn -= 250
        if(self.headTurn < 4100):
            self.headTurn = 4100
        self.tango.setTarget(HEADTURN, self.headTurn)
        print('TURN HEAD RIGHT')  
    def tiltUp(self):
        self.headTilt += 250
        if(self.headTilt > 7900):
            self.headTilt = 7900
        self.tango.setTarget(HEADTILT, self.headTilt)
    def tiltDown(self):
        self.headTilt -= 250
        if(self.headTilt < 4100):
            self.headTilt = 4100
        self.tango.setTarget(HEADTILT, self.headTilt)
    def left(self):
        self.turn += 1100
        self.tango.setTarget(TURN, self.turn)
        print("LEFT")
        time.sleep(.25)
        self.turn -= 1100
        self.tango.setTarget(TURN, self.turn)
    def right(self):
        self.turn -= 1100
        self.tango.setTarget(TURN, self.turn)
        print("RIGHT")
        time.sleep(.25)
        self.turn += 1100
        self.tango.setTarget(TURN, self.turn)
    def forward(self):
        self.motors = 4800
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
        time.sleep(2.6)
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

    def nodHead(self):
        self.tiltUp()
        time.sleep(0.5)
        self.tiltDown()
        time.sleep(0.5)
        self.tiltUp()
        time.sleep(0.5)
        self.tiltDown()
        time.sleep(0.5)

    def resetWaist(self):
        self.body = 5800
        self.tango.setTarget(BODY, self.body)
    def waistLeft(self):
        self.body += 500
        if(self.body > 7900):
            self.body = 7900
        self.tango.setTarget(BODY, self.body)
        print("waist right")
    def waistRight(self):
        self.body -= 500
        if(self.body < 4100):
            self.body = 4100
        self.tango.setTarget(BODY, self.body)
        print("waist left")

    def setWaist(self, num):
        if num > 7900:
            num = 7900
        if num < 4100:
            num - 4100
        self.body = num
        self.tango.setTarget(BODY, self.body)
        
    #ARMS?
    def shoulderUp(self):
        self.shoulder += 500
        if(self.shoulder > 7900):
            self.shoulder = 7900
        self.tango.setTarget(SHOULDER, self.shoulder)
    def shoulderDown(self):
        self.shoulder -= 500
        if(self.shoulder > 1510):
            self.shoulder = 1510
        self.tango.setTarget(SHOULDER, self.shoulder)
    def elbowOpen(self):
        self.elbow += 500
        if(self.elbow > 7900):
            self.elbow = 7900
        self.tango.setTarget(ELBOW, self.elbow)
    def elbowClose(self):
        self.elbow -= 1000
        if(self.elbow > 1510):
            self.elbow = 1510
        self.tango.setTarget(ELBOW, self.elbow)
    def handClose(self):
        self.hand += 1300
        if(self.hand > 7900):
            self.hand = 7900
        self.tango.setTarget(HAND, self.hand)
    def handOpen(self):
        self.hand -= 1300
        if(self.hand > 1510):
            self.hand = 1510
        self.tango.setTarget(HAND, self.hand)
    def handReset(self):
        self.hand = 5000
        self.tango.setTarget(HAND, self.hand)
    def elbowUpMax(self):
        self.elbow = 7900
        self.tango.setTarget(ELBOW, self.elbow)
    def elbowReset(self):
        self.elbow = 6000
        self.tango.setTarget(ELBOW, self.elbow)
        
    def searchForFaces(self):
        print("searching")
        headTilt = self.headTilt
        headTurn = self.headTurn
        if(headTilt >= 7000):
            self.resetSearchTilt = True
        if(headTilt <= 5000):
            self.resetSearchTilt = False

        resetPan = self.resetSearchPan
        resetTilt = self.resetSearchTilt

        if(headTurn < 7900 and resetPan is False):
            self.turnLeft()
        elif(headTurn >= 7900 and resetPan is False):
            self.resetSearchPan = True
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnRight()
        elif(headTurn > 4100 and resetPan is True):
            self.turnRight()
        elif(headTurn <= 4100 and resetPan is True):
            self.resetSearchPan = False
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnLeft()

    
    def tiltHeadDownMax(self):
        self.headTilt = 4400
        self.tango.setTarget(HEADTILT, self.headTilt)


    def alignMoveToFace(self, face):
        global faceSize, faceSizeTolerance, faceTolerance, centerx, centery
        largest = 0
        current = None
        for (x,y,w,h) in face:
            if(w*h) > largest:
                largest = w*h
                current = [int(x+(w/2)), int(y+(h/2))]
        if(current[1] - centery > faceTolerance):
            self.tiltDown()
        elif(current[1] - centery < -faceTolerance):
            self.tiltUp()
        else:
            if(current[0] - centerx > faceTolerance): #too far right
                print("too far left, turn body right")
                self.right()
            elif(centerx - current[0] > faceTolerance): #too far left
                print("too far right, turn body left")
                self.left()
            elif(self.headTurn < 5900):
                print("head is turned right, turning it left now yeet")
                self.turnLeft()
            elif(self.headTurn > 6100):
                print("head is turned left, turning it right now")
                self.turnRight()
            else:
                if(largest > faceSize + faceSizeTolerance): #human too big
                    print("Go backward")
                    self.backward()
                elif(largest < faceSize - faceSizeTolerance):
                    print("Go forward")
                    self.forward()
                else:
                    return False
        return True


    def alignFace(self, face):
        global faceSize, faceSizeTolerance, faceTolerance, centerx, centery
        largest = 0
        current = None
        for (x,y,w,h) in face:
            if(w*h) > largest:
                largest = w*h
                current = [int(x+(w/2)), int(y+(h/2))]
        if(current[1] - centery > faceTolerance):
            self.tiltDown()
        elif(current[1] - centery < -faceTolerance):
            self.tiltUp()
        elif(current[0] - centerx > faceTolerance): #too far right
            print("too far left, turn head right")
            self.turnRight()
        elif(centerx - current[0] > faceTolerance): #too far left
            print("too far right, turn head left")
            self.turnLeft()

controller = Control()