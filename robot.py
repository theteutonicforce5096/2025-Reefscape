# TODO: insert robot code here

import commands2
from robot_container import RobotContainer

class ReefScapeRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer()
    
    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
    
    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.config_button_bindings_teleop()