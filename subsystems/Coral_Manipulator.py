import wpilib
from wpilib import Joystick
from wpilib import Timer
import rev
import wpimath
from networktables import NetworkTables
from rev import ClosedLoopConfig
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

class Coral_Manipulator:
   
    def __init__(self):
        
        #Configuring Motor
        self.sucker_config = rev.SparkBaseConfig()
        self.sucker = rev.SparkMax(
            51, rev.SparkMax.MotorType.kBrushless
        )
        self.sucker.configure(
            self.sucker_config,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters,
        )
        


    def intake(self):
        self.sucker.set(-0.25)
        
        
    def expel(self):
        self.sucker.set(.25)
        # We might need to make this value higher because it might be too slow
        
        
    def stop(self):
        print("Sanity Check 2")
        self.sucker.set(0)
        
