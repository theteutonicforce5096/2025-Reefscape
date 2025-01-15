import commands2
from robot_container import RobotContainer

class Reefscape2025(commands2.TimedCommandRobot):
    """
    Robot for Team 5096.
    """

    def robotInit(self):
        self.container = RobotContainer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def teleopInit(self):
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_teleop()

    def testInit(self):
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_test()
