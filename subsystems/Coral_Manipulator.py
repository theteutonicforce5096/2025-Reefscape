import wpilib
from wpilib import Joystick
from wpilib import Servo
import wpimath
import wpimath.units
import robot


class coral_manipulator:

    def __init__(self):
        self.WAcontroller = wpilib.Servo(9)
        # The Actuonix L16 expects pulse widths between 1000 µs and 2000 µs
        self.WAcontroller.setBounds(
            max=2000,
            deadbandMax=1500,
            center=1500,
            deadbandMin= 1500,
            min=1000)
        self.wide_limits = False

    def PSA(self):
         # Position to scoring angle oh yeah!!
        new_position = 0.0 if self.wide_limits else 0.25
        print(f"{new_position:0.2f}")
        self.WAcontroller.setPosition(new_position)

    def PLA(self):
          # Position to Loading Angle
        new_position = 1.0 if self.wide_limits else 0.75
        print(f"{new_position:0.2f}")
        self.WAcontroller.setPosition(new_position)

    def RESET(self):
        print("RESET")
        self.wide_limits = not self.wide_limits
