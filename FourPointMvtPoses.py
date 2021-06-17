import numpy as np
import math
from adafruit_servokit import ServoKit

def inverseKin(x,y,z,Lua,Lfa):
    D = math.sqrt((y**2)+(z**2))
    A = (1/(Lua*2))*((Lfa**2)-(Lua**2)-(x**2)-(D**2))
    R = math.sqrt((x**2)+(D**2))
    alpha = math.atan(x/(-1*D))
    omega = math.asin(A/R) + alpha
    theta = omega + math.pi
    phi =(-1* math.asin((1/Lfa)*(-1*D+Lua*math.sin(-omega))))-omega
    gamma = math.asin(z/D)
    return theta,phi,gamma;


class movementPoses:
    def __init__(self):
        self.thetaOne = 90
        self.phiOne = 90
        self.thetaTwo = 90
        self.phiTwo = 90
        self.gammaOne = 90
        self.gammaTwo = 90
        self.kit = ServoKit(channels=16)
        self.angles = np.zeros((3,4))
        self.neutralAngles = np.array([[90,90,90,90],[90,90,90,90],[90,90,90,90]])
        self.counter = 0
        self.strideLength = 2
        self.strideHeight = .5
        self.COMHeight = 5
        self.upperarmLength = 4
        self.forearmLength = 5
        self.legForward = inverseKin((self.strideLength/2),(-1*self.COMHeight),0,self.upperarmLength,self.forearmLength)
        self.legCenter = inverseKin(0,(-1*self.COMHeight),0,self.upperarmLength,self.forearmLength)
        self.legBack = inverseKin((-1*self.strideLength/2),(-1*self.COMHeight),0,self.upperarmLength,self.forearmLength)
        self.legUp = inverseKin(0,(-1*self.COMHeight+self.strideHeight),0,self.upperarmLength,self.forearmLength)
        self.legRight = inverseKin(0,-1*self.COMHeight,self.strideLength/2,self.upperarmLength,self.forearmLength)
        self.legLeft = inverseKin(0,-1*self.COMHeight,-1*self.strideLength/2,self.upperarmLength,self.forearmLength)
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = self.neutralAngles[j,i]
                self.angles[j,i] = self.neutralAngles[j,i]
                self.counter += 1

    def forwardOne(self):
        self.thetaOne = math.degrees(self.legForward[0])
        self.phiOne = math.degrees(self.legForward[1])
        self.thetaTwo = math.degrees(self.legBack[0])
        self.phiTwo = math.degrees(self.legBack[1])
        self.angles = np.array([[90,90,90,90],[180-self.thetaOne,self.thetaTwo,180-self.thetaTwo,self.thetaOne],[180-self.phiOne,self.phiTwo,180-self.phiTwo,self.phiOne]])
        self.counter = 0
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = self.angles[j,i]
                self.counter += 1

    def midStepOne(self):
        self.thetaOne = math.degrees(self.legCenter[0])
        self.phiOne = math.degrees(self.legCenter[1])
        self.thetaTwo = math.degrees(self.legUp[0])
        self.phiTwo = math.degrees(self.legUp[1])
        self.angles = np.array([[90,90,90,90],[180-self.thetaOne,self.thetaTwo,180-self.thetaTwo,self.thetaOne],[180-self.phiOne,self.phiTwo,180-self.phiTwo,self.phiOne]])
        self.counter = 0
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = self.angles[j,i]
                self.counter += 1

    def forwardTwo(self):
        self.thetaOne = math.degrees(self.legBack[0])
        self.phiOne = math.degrees(self.legBack[1])
        self.thetaTwo = math.degrees(self.legForward[0])
        self.phiTwo = math.degrees(self.legForward[1])
        self.angles = np.array([[90,90,90,90],[180-self.thetaOne,self.thetaTwo,180-self.thetaTwo,self.thetaOne],[180-self.phiOne,self.phiTwo,180-self.phiTwo,self.phiOne]])
        self.counter = 0
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = self.angles[j,i]
                self.counter += 1

    def midStepTwo(self):
        self.thetaOne = math.degrees(self.legUp[0])
        self.phiOne = math.degrees(self.legUp[1])
        self.thetaTwo = math.degrees(self.legCenter[0])
        self.phiTwo = math.degrees(self.legCenter[1])
        self.angles = np.array([[90,90,90,90],[180-self.thetaOne,self.thetaTwo,180-self.thetaTwo,self.thetaOne],[180-self.phiOne,self.phiTwo,180-self.phiTwo,self.phiOne]])
        self.counter = 0
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = self.angles[j,i]
                self.counter += 1

    def sideStepOne(self):
        self.thetaOne = math.degrees(self.legRight[0])
        self.phiOne = math.degrees(self.legRight[1])
        self.thetaTwo = math.degrees(self.legLeft[0])
        self.phiTwo = math.degrees(self.legLeft[1])
        self.gammaOne = math.degrees(self.legRight[2])
        self.gammaTwo = math.degrees(self.legLeft[2])
        self.angles = np.array([[90+self.gammaOne,90+self.gammaTwo,90+self.gammaTwo,90+self.gammaOne],[180-self.thetaOne,self.thetaTwo,180-self.thetaTwo,self.thetaOne],[180-self.phiOne,self.phiTwo,180-self.phiTwo,self.phiOne]])
        self.counter = 0
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = self.angles[j,i]
                self.counter += 1

    def sideStepTwo(self):
        self.thetaOne = math.degrees(self.legLeft[0])
        self.phiOne = math.degrees(self.legLeft[1])
        self.thetaTwo = math.degrees(self.legRight[0])
        self.phiTwo = math.degrees(self.legRight[1])
        self.gammaOne = math.degrees(self.legLeft[2])
        self.gammaTwo = math.degrees(self.legRight[2])
        self.angles = np.array([[90+self.gammaOne,90+self.gammaTwo,90+self.gammaTwo,90+self.gammaOne],[180-self.thetaOne,self.thetaTwo,180-self.thetaTwo,self.thetaOne],[180-self.phiOne,self.phiTwo,180-self.phiTwo,self.phiOne]])
        self.counter = 0
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = self.angles[j,i]
                self.counter += 1
