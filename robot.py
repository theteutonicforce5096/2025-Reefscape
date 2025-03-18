from enum import Enum, auto
import wpilib
from wpilib import Joystick
from networktables import NetworkTables
from subsystems import climb_mechanism
import wpimath
import rev


class ReefscapeRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.pxn_fightstick = wpilib.Joystick(0)

        # Initializing Servos
        self.andyMark = wpilib.Servo(0)
        self.andyMark.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )

        # Initializing Motor
        self.ClimberMotorLeft = rev.SparkMax(49, rev.SparkMax.MotorType.kBrushless)
        self.ClimberMotorLeft.set(0.0)

        ClimbMotorLeftConfig = rev.SparkBaseConfig()
        ClimbMotorLeftConfig.inverted(True)
        self.ClimberMotorLeft.configure(
            ClimbMotorLeftConfig,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )

        # Initializing girly pop climbgal
        self.Climbgal = climb_mechanism.climb_mechanism(
            self.andyMark, self.ClimberMotorLeft
        )

        # Network tables are used for Test Mode
        NetworkTables.initialize(server="roborio-5096-frc.local")
        self.sd_table = NetworkTables.getTable("SmartDashboard")

    def teleopInit(self):
        pass

    def teleopPeriodic(self):

        # Calling the methods made in Climbguy.py

        if self.pxn_fightstick.getRawButtonPressed(3):
            self.Climbgal.__getHomePosition__()

        self.Climbgal.periodic()

    def teleopExit(self):
        pass


    ### TEST MODE STUFF ###

    class tm_ClimberSelect(Enum):
        LEFT = auto()
        RIGHT = auto()

    def testInit(self):

        # Testing the encoder
        # self.encodingTimer.restart()

        # The following code has to do with servcing the controls on the Test Mode screen of the custom dashboard

        # Determine which climber is selected
        self.tm_which_climber = self.tm_ClimberSelect.LEFT \
            if self.sd_table.getBoolean('climber_LR_switch', False) \
                else self.tm_ClimberSelect.RIGHT
        selected_climber = self.Climbgal
        #TODO: Need to handle switching to the right climber

        # Make sure motors are stopped
        self.Climbgal.stop()
        #TODO: Command the other climber motor to stop
        
        # Determine the initial position of the ratchet switch
        if self.sd_table.getBoolean('ratchet_engage', False):
            selected_climber.__engageRatchet__()
        else:
            selected_climber.__disengageRatchet__()

        return super().testInit()

    def testPeriodic(self):

        if self.pxn_fightstick.getRawButtonPressed(4):
            #TODO: Comment this out for now, as we are not using the encoder
            self.Climbgal.findHomePosition()

        if self.pxn_fightstick.getRawButtonPressed(2):
            self.Climbgal.stop()

        self.Climbgal.testPeriodic()

        # Below this point, the code is servicing the controls on the Test Mode screen of the custom dashboard

        # Determine which climber is selected
        which_climber = self.tm_ClimberSelect.LEFT \
            if self.sd_table.getBoolean('climber_LR_switch', False) \
                else self.tm_ClimberSelect.RIGHT
        
        # If the operator has switched the climber, stop the current climber
        if which_climber != self.tm_which_climber:
            pass
            #TODO: There should be code here for stopping the current climber when switching to the other one
            self.tm_which_climber = which_climber
        
        # Grab a reference to the selected climber
        selected_climber = self.Climbgal
        #TODO: Need to handle switching to the other climber

        # Handle the ratchet
        if self.sd_table.getBoolean('ratchet_engage', False):
            selected_climber.__engageRatchet__()
        else:
            selected_climber.__disengageRatchet__()

        # Display the absolute encoder value
        #TODO: Need to monitor the abs enc value and pass it to network tables

        # Monitor the joystick and turn them into motor commands
        #TODO: Need to set up a joystick (in robotInit()) and monitor it here




if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)
