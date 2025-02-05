print("skibidi toilet ohio rizz")
import wpilib
from wpilib import Joystick
from wpilib import Servo
import robot



class coral_manipulator():

    def __init__(self):
        self.WAcontroller = wpilib.Servo(0)
        self.WAcontroller.setPosition(0)
       
        
    def PSA(self):
        #Position to scoring angle oh yeah!!
        self.WAcontroller.setPosition(.5)
    
    def PLA(self):
        #Position to Loading Angle
        self.WAcontroller.setPosition(.75)
    def RESET(self):
        self.WAcontroller.setPosition(.25)
    
    
    
    
    

        