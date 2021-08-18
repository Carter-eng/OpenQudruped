#Robot physical properties in inches
import numpy as np

class Properties:
    def __init__(self):
        self.upperarmLength = 4
        self.forearmLength = 5
        self.bodyWidth = 5
        self.bodyLength = 7
        self.baseStandingHeight = 5
        self.restingAngles = np.array([[90,90,90,90],[90,90,90,90],[90,90,90,90]])
