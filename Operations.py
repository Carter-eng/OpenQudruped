"""
These are operations for use by the OpenQuadruped
Included here are inverse kinematics, orientation
calculations, and hardware handling the RobotProperties.py
script contains the dimensions used for the robot
created by Carter Berlind for the BU Robotics lab 2021
"""

import numpy as np
import math
from RobotProperties import Properties
from adafruit_servokit import ServoKit

#The inputs are the foot position and outputs the servo leg angles
#note that the origin is considered to be the shoulder and
#these consider a coordinate system with positive y
#going upwards, and positive x going forward and positivez going
#out of the side of the robot
class inverseKinematics:
    def __init__(self):
        self.Properties = Properties()
        self.Lua = self.Properties.upperarmLength
        self.Lfa = self.Properties.forearmLength

    def InverseKin2D(self,x,y):
        A = (1/(self.Lua*2))*(self.(Lfa^2)-(self.Lua^2)-(x^2)-(y^2))
        R = math.sqrt((x^2)+(y^2))
        alpha = math.atan(x/y)
        omega = math.asin(A/R) + alpha
        theta = omega + math.pi
        phi =(-1* math.asin((1/self.Lfa)*(y+self.Lua*math.sin(-omega))))-omega
        return [theta,phi];

    def InverseKin3D(self,x,y,z):
        D = math.sqrt((y**2)+(z**2))
        A = (1/(Lua*2))*((Lfa**2)-(Lua**2)-(x**2)-(D**2))
        R = math.sqrt((x**2)+(D**2))
        alpha = math.atan(x/(-1*D))
        omega = math.asin(A/R) + alpha
        theta = omega + math.pi
        phi =(-1* math.asin((1/Lfa)*(-1*D+Lua*math.sin(-omega))))-omega
        gamma = math.asin(z/D)
        return [gamma,theta,phi];

class OrientationHandler:
    def __init__(self):
        self.Properties = Properties()
        self.bodyLength = self.Properties.bodyLength
        self.bodyWidth = self.Properties.bodyWidth
        self.yawRadius = math.sqrt((self.bodyLength**2)+(self.bodyWidth**2))/2

    def findPitch(self,frontHeight,backHeight):
        heightDifference = frontHeight - backHeight
        pitch = math.asin(heightDifference/self.bodyLength)
        return pitch;

    def pitch(self,pitch):
        heightDifference = self.bodyLength*math.sin(pitch)
        frontHeightChange =  heightDifference/2
        backHeightChange =  -1*heightDifference/2
        return [frontHeightChange,backHeightChange];

    def findRoll(self,rightHeight,leftHeight):
        heightDifference = leftHeight - rightHeight
        roll = math.asin(heightDifference/self.bodyWidth)
        return roll;

    def roll(self,roll):
        heightDifference = self.bodyWidth*math.sin(roll)
        rihgHeightChange = -1*heightDifference/2
        leftHeightChange = heightDifference/2
        return [rihgHeightChange,leftHeightChange];

    def findYawFromZ(self,yawChangeZ):
        yaw = math.asin(yawChangeZ/self.yawRadius)
        return yaw;

    def findYawFromX(self,yawChangeX):
        yaw = math.asin(1-(yawChangeX/self.yawRadius))
        return yaw;

    def staticYaw(self,yaw):
        frontRightChangeZ = self.yawRadius*math.sin(yaw)
        frontRightChangeX = -1*self.yawRadius*(1-math.cos(yaw))
        frontLeftChangeZ = -1*self.yawRadius*math.sin(yaw)
        frontLeftChangeX = -1*self.yawRadius*(1-math.cos(yaw))
        backRightChangeZ = -1*self.yawRadius*math.sin(yaw)
        backRightChangeX = self.yawRadius*(1-math.cos(yaw))
        backLeftChangeZ = self.yawRadius*math.sin(yaw)
        backLeftChangeX = self.yawRadius*(1-math.cos(yaw))
        return [frontRightChangeZ,frontRightChangeX,frontLeftChangeZ,frontLeftChangeX,backRightChangeZ,backRightChangeX,backLeftChangeZ,backLeftChangeX];

class hardwareHandler:
    def __init__(self):
        self.kit = ServoKit(channels=16)
        self.counter = 0

    def anglePublisher(self,angles):
        for i in range (4):
            for j in range (3):
                self.kit.servo[self.counter].angle = angles[j,i]
                self.counter += 1
