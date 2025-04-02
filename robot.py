import commands2
from robot_container import RobotContainer

class ReefscapeRobot(commands2.TimedCommandRobot):
    """
    Robot for Team 5096.
    """
        
    def robotInit(self):
        self.container = RobotContainer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def autonomousInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def autonomousPeriodic(self):
        self.container.elevator.run_pid()

    def autonomousExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_teleop()

    def teleopPeriodic(self):
        self.container.elevator.run_pid()

    def teleopExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_test()

    def testPeriodic(self):
        pass

    def testExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()