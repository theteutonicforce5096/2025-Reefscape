import commands2
from motor import Motor
import commands2.button

class RobotContainer():
    def __init__(self):
        self.controller = commands2.button.CommandXboxController(0)
        self.motor = Motor(0)

    def config_button_bindings_teleop(self):
        (self.controller.rightBumper() & self.controller.leftBumper()).onFalse(self.motor.runOnce(lambda: self.motor.set_speed(0)))
        (self.controller.rightBumper() & self.controller.leftBumper()).onTrue(self.motor.runOnce(lambda: self.motor.set_speed(0.1)))