import commands2
from motor import motor
import commands2.button


class RobotContainer():
    def __init__(self):
        self.controller = commands2.button.CommandXboxController(0)
        self.drivetrain = motor()

    def config_button_bindings_teleop(self):
        (self.controller.rightBumper() & self.controller.leftBumper()).onFalse(self.drivetrain.runOnce(lambda: print('jimmy said bye :(')))
        (self.controller.rightBumper() & self.controller.leftBumper()).onTrue(self.drivetrain.runOnce(lambda: print('jimmy said hi')))