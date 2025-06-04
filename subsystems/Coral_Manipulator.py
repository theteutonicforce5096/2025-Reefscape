import wpilib
from wpilib import Joystick
from wpilib import Timer
import rev
from networktables import NetworkTables
from rev import ClosedLoopConfig
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class coral_manipulator:
   
    def __init__(self):
        
        #Configuring Motor
        self.sucker_config = rev.SparkBaseConfig
        self.sucker = rev.SparkMax(
            50, rev.SparkMax.MotorType.kBrushless
        )
        self.sucker.configure(
            self.sucker_config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )
        #Configuring Timer
        self.intake_timer = wpilib.Timer


    def intake(self):
        self.sucker.set(.25)
        self.intake_timer.restart
        if self.intake_timer.hasElapsed(True, 3):
            self.sucker.set(0)

    def stop(self):
        self.sucker.set(0)
