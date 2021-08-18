import numpy as np
import math
import RobotProperties
from RobotProperties import Properties
import Operations
from Operations import inverseKinematics, OrientationHandler

class legPositioner:
    def __init__():
        self.Properties = Properties()
        self.inverseKin = inverseKinematics()
        self.OrientationCalc = OrientationHandler()

    def forwardOne(self,speedMultiplier,orientation):
        rollChanges = self.OrientationCalc.roll(orientation[0])
        pitchChanges = self.OrientationCalc.pitch(orientation[1])
        yawChanges = self.OrientationCalc.staticYaw(orientaton[2])
        frontRightX = yawChanges[1]+(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(pitch)
        frontRightY = (-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        frontRightZ = yawChanges[0]-(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(roll)
        frontRight = self.inverseKin.InverseKin3D(frontRightX,frontRightY,frontRightZ)
        frontLeftX = yawChanges[3]-(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(pitch)
        frontLeftY = (-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        frontLeftZ = yawChanges[2]+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin()
        frontLeft = self.inverseKin.InverseKin3D(frontLeftX,frontLeftY,frontLeftZ)
        backRightX = yawChanges[5]-(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(pitch)
        backRightY = (-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        backRightZ = yawChanges[4]-(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(roll)
        backRight = self.inverseKin.InverseKin3D(backRightX,backRightY,backRightZ)
        backLeftX = yawChanges[7]+(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(pitch)
        backLeftY = (-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        backLeftZ = yawChanges[6]+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(roll)
        backLeft = self.inverseKin.InverseKin3D(backLeftX,backLeftY,backLeftZ)
        angles = np.array([[frontRight[0],frontLeft[0],backRight[0],backLeft[0]],[frontRight[1],frontLeft[1],backRight[1],backLeft[1]],[frontRight[2],frontLeft[2],backRight[2],backLeft[2]]])
        return angles;

    def midstepOne(self,orientation):
        rollChanges = self.OrientationCalc.roll(orientation[0])
        pitchChanges = self.OrientationCalc.pitch(orientation[1])
        yawChanges = self.OrientationCalc.staticYaw(orientaton[2])
        frontRightX = yawChanges[1]+(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(pitch)
        frontRightY = (-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        frontRightZ = yawChanges[0]-(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(roll)
        frontRight = self.inverseKin.InverseKin3D(frontRightX,frontRightY,frontRightZ)
        frontLeftX = yawChanges[3]+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight+self.Properties.baseStrideHeight)*math.sin(pitch)
        frontLeftY = (-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeigh+self.Properties.baseStrideHeight)*math.cos(pitch)*math.cos(roll)
        frontLeftZ = yawChanges[2]+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight+self.Properties.baseStrideHeight)*math.sin()
        frontLeft = self.inverseKin.InverseKin3D(frontLeftX,frontLeftY,frontLeftZ)
        backRightX = yawChanges[5]+(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight+self.Properties.baseStrideHeight)*math.sin(pitch)
        backRightY = (-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight+self.Properties.baseStrideHeight)*math.cos(pitch)*math.cos(roll)
        backRightZ = yawChanges[4]-(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight+self.Properties.baseStrideHeight)*math.sin(roll)
        backRight = self.inverseKin.InverseKin3D(backRightX,backRightY,backRightZ)
        backLeftX = yawChanges[7]+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(pitch)
        backLeftY = (-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        backLeftZ = yawChanges[6]+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(roll)
        backLeft = self.inverseKin.InverseKin3D(backLeftX,backLeftY,backLeftZ)
        angles = np.array([[frontRight[0],frontLeft[0],backRight[0],backLeft[0]],[frontRight[1],frontLeft[1],backRight[1],backLeft[1]],[frontRight[2],frontLeft[2],backRight[2],backLeft[2]]])
        return angles;

    def forwardTwo(self,speedMultiplier,orientation):
        rollChanges = self.OrientationCalc.roll(orientation[0])
        pitchChanges = self.OrientationCalc.pitch(orientation[1])
        yawChanges = self.OrientationCalc.staticYaw(orientaton[2])
        frontRightX = yawChanges[1]-(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(pitch)
        frontRightY = (-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        frontRightZ = yawChanges[0]-(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(roll)
        frontRight = self.inverseKin.InverseKin3D(frontRightX,frontRightY,frontRightZ)
        frontLeftX = yawChanges[3]+(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(pitch)
        frontLeftY = (-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        frontLeftZ = yawChanges[2]+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin()
        frontLeft = self.inverseKin.InverseKin3D(frontLeftX,frontLeftY,frontLeftZ)
        backRightX = yawChanges[5]+(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(pitch)
        backRightY = (-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        backRightZ = yawChanges[4]-(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(roll)
        backRight = self.inverseKin.InverseKin3D(backRightX,backRightY,backRightZ)
        backLeftX = yawChanges[7]-(self.Properties.maxStrideLength*speedMultiplier/2)+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(pitch)
        backLeftY = (-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        backLeftZ = yawChanges[6]+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(roll)
        backLeft = self.inverseKin.InverseKin3D(backLeftX,backLeftY,backLeftZ)
        angles = np.array([[frontRight[0],frontLeft[0],backRight[0],backLeft[0]],[frontRight[1],frontLeft[1],backRight[1],backLeft[1]],[frontRight[2],frontLeft[2],backRight[2],backLeft[2]]])
        return angles;

    def midstepTwo(self,orientation):
        rollChanges = self.OrientationCalc.roll(orientation[0])
        pitchChanges = self.OrientationCalc.pitch(orientation[1])
        yawChanges = self.OrientationCalc.staticYaw(orientaton[2])
        frontRightX = yawChanges[1]+(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight++self.Properties.baseStrideHeight)*math.sin(pitch)
        frontRightY = (-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight++self.Properties.baseStrideHeight)*math.cos(pitch)*math.cos(roll)
        frontRightZ = yawChanges[0]-(-1*rollChanges[0]-pitchChanges[0]-self.Properties.baseStandingHeight++self.Properties.baseStrideHeight)*math.sin(roll)
        frontRight = self.inverseKin.InverseKin3D(frontRightX,frontRightY,frontRightZ)
        frontLeftX = yawChanges[3]+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin(pitch)
        frontLeftY = (-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        frontLeftZ = yawChanges[2]+(-1*rollChanges[1]-pitchChanges[0]-self.Properties.baseStandingHeight)*math.sin()
        frontLeft = self.inverseKin.InverseKin3D(frontLeftX,frontLeftY,frontLeftZ)
        backRightX = yawChanges[5]+(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(pitch)
        backRightY = (-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.cos(pitch)*math.cos(roll)
        backRightZ = yawChanges[4]-(-1*rollChanges[0]-pitchChanges[1]-self.Properties.baseStandingHeight)*math.sin(roll)
        backRight = self.inverseKin.InverseKin3D(backRightX,backRightY,backRightZ)
        backLeftX = yawChanges[7]+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight++self.Properties.baseStrideHeight)*math.sin(pitch)
        backLeftY = (-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight++self.Properties.baseStrideHeight)*math.cos(pitch)*math.cos(roll)
        backLeftZ = yawChanges[6]+(-1*rollChanges[1]-pitchChanges[1]-self.Properties.baseStandingHeight++self.Properties.baseStrideHeight)*math.sin(roll)
        backLeft = self.inverseKin.InverseKin3D(backLeftX,backLeftY,backLeftZ)
        angles = np.array([[frontRight[0],frontLeft[0],backRight[0],backLeft[0]],[frontRight[1],frontLeft[1],backRight[1],backLeft[1]],[frontRight[2],frontLeft[2],backRight[2],backLeft[2]]])
        return angles;
