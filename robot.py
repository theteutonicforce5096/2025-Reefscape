import commands2
import wpilib
from networktables import NetworkTables

from robot_container import RobotContainer

from subsystems import climb_mechanism

class ReefscapeRobot(commands2.TimedCommandRobot):
    """
    Robot for Team 5096.
    """
    
    def robotInit(self):
        self.container = RobotContainer()

        RATCHET_SERVO_L_ID = 0
        RATCHET_SERVO_R_ID = 1
        CLIMB_MOTOR_L_ID = 61
        CLIMB_MOTOR_R_ID = 62

        self.pxn_fightstick = wpilib.Joystick(2)
        self.goodStick = wpilib.XboxController(1)

        # Initializing girly pop climbgal left
        self.ABSOLUTE_MAX_POSITION_L = 0.350204616785049
        self.ABSOLUTE_MIN_POSITION_L = -0.287883698940277
        # Range of 0.612
        self.Climbgal_L = climb_mechanism.climb_mechanism(
            RATCHET_SERVO_L_ID,
            CLIMB_MOTOR_L_ID,
            "climber_L",
            self.ABSOLUTE_MIN_POSITION_L,
            self.ABSOLUTE_MAX_POSITION_L,
        )

        # Initializing girly pop climbgal right
        self.ABSOLUTE_MAX_POSITION_R = 0.800914824008942
        self.ABSOLUTE_MIN_POSITION_R = 0.14428836107254
        # Range of 0.606
        self.Climbgal_R = climb_mechanism.climb_mechanism(
            RATCHET_SERVO_R_ID,
            CLIMB_MOTOR_R_ID,
            "climber_R",
            self.ABSOLUTE_MIN_POSITION_R,
            self.ABSOLUTE_MAX_POSITION_R,
        )

        # Network tables are used for Test Mode
        NetworkTables.initialize(server="roborio-5096-frc.local")
        self.sd_table = NetworkTables.getTable("SmartDashboard")
    
    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        
    def autonomousInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_auto()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_teleop()
        
        # self.Climbgal_L.teleopInit()
        # self.Climbgal_R.teleopInit()

        # self.Climbgal_L.reset()
        # self.Climbgal_R.reset()

    def teleopPeriodic(self):
        self.container.elevator.run_pid()
        # self.Climbgal_L.periodic()
        # self.Climbgal_R.periodic()

        # if self.pxn_fightstick.getRawButtonPressed(9):
        #     self.Climbgal_L.climb()
        #     self.Climbgal_R.climb()
        # if self.pxn_fightstick.getRawButtonPressed(10):
        #     self.Climbgal_L.reset()
        #     self.Climbgal_R.reset()

    def teleopExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_test()
        
        self.Climbgal_L.stop()
        self.Climbgal_R.stop()
        self.Climbgal_R.testInit()
        self.Climbgal_L.testInit()

    def testPeriodic(self):    
        self.Climbgal_L.testPeriodic()
        self.Climbgal_R.testPeriodic()

        # Left Thumbstick
        joystickAxis_L = -self.goodStick.getLeftY()
        self.Climbgal_L.motorDirect(joystickAxis_L)
        # Right Thumbstick
        joystickAxis_R = -self.goodStick.getRightY()
        self.Climbgal_R.motorDirect(joystickAxis_R)

        # Making the encoder values corresponding to the drivers station or smth
        self.sd_table.putNumber("climber_R_motor_cmd", joystickAxis_R)
        self.sd_table.putNumber("climber_L_motor_cmd", joystickAxis_L)

        ### DISENGAGING AND ENGAGING RATCHET??? ###
        ratchet_engage_L = self.sd_table.getBoolean("ratchet_engage_L", False)
        ratchet_engage_R = self.sd_table.getBoolean("ratchet_engage_R", False)
        # Getting T/F from Network table
        if ratchet_engage_L == True:
            self.Climbgal_L.__engageRatchet__()
        else:
            self.Climbgal_L.__disengageRatchet__()

        if ratchet_engage_R == True:
            self.Climbgal_R.__engageRatchet__()
        else:
            self.Climbgal_R.__disengageRatchet__()

    def testExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)