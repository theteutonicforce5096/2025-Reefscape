import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath.units
from wpilib import Timer
import rev
from networktables import NetworkTables
from rev import ClosedLoopConfig
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class coral_manipulator:
   
    def __init__(self):
        self.sucker_config = rev.SparkBaseConfig
        self.sucker = rev.SparkMax(
            50, rev.SparkMax.MotorType.kBrushless
        )
        self.sucker.configure(
            self.sucker_config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )
    def intake(self):
        self.sucker.set(.25)
