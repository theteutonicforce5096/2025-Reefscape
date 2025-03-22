from enum import Enum, auto
import wpilib
from wpilib import Joystick
from networktables import NetworkTables
from subsystems import climb_mechanism
import wpimath
import rev


class ReefscapeRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.pxn_fightstick = wpilib.Joystick(1)
        self.goodStick = wpilib.XboxController(0)

        # Initializing Left Servo
        self.andyMark_L = wpilib.Servo(0)
        self.andyMark_L.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )

        # Initializing Right Servo
        self.andyMark_R = wpilib.Servo(1)
        self.andyMark_R.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )

        # Initializing Left Encoder
        self.encoderDude_L = wpilib.DutyCycleEncoder(0)
        self.encoderDude_L.setAssumedFrequency(wpimath.units.hertz(975.6))
        self.encoderDude_L.setDutyCycleRange(min=1 / 1025, max=1024 / 1025)
        self.encoderDude_L.setInverted(True)

        # Initializing Right Encoder
        self.encoderDude_R = wpilib.DutyCycleEncoder(1)
        self.encoderDude_R.setAssumedFrequency(wpimath.units.hertz(975.6))
        self.encoderDude_R.setDutyCycleRange(min=1 / 1025, max=1024 / 1025)
        self.encoderDude_R.setInverted(True)

        # Initializing Left Motor
        self.ClimberMotorLeft = rev.SparkMax(49, rev.SparkMax.MotorType.kBrushless)
        self.ClimberMotorLeft.set(0.0)

        ClimberMotorLeftConfig = rev.SparkBaseConfig()
        ClimberMotorLeftConfig.inverted(True)
        self.ClimberMotorLeft.configure(
            ClimberMotorLeftConfig,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )

        # Initializing Right Motor
        self.ClimberMotorRight = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)
        self.ClimberMotorRight.set(0.0)

        ClimberMotorRightConfig = rev.SparkBaseConfig()
        ClimberMotorRightConfig.inverted(True)
        self.ClimberMotorRight.configure(
            ClimberMotorRightConfig,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )

        # Initializing girly pop climbgal left
        self.Climbgal_L = climb_mechanism(
            self.andyMark_L, self.ClimberMotorLeft, self.encoderDude_L
        )

        # Initializing girly pop climbgal right
        self.Climbgal_R = climb_mechanism(
            self.andyMark_R, self.ClimberMotorRight, self.encoderDude_R
        )

        # Network tables are used for Test Mode
        NetworkTables.initialize(server="roborio-5096-frc.local")
        self.sd_table = NetworkTables.getTable("SmartDashboard")

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Calling 'Get absolute encoder'
        self.Climbgal_L.AbsEncoderVal()

        # Calling the methods made in Climbguy.py

        if self.pxn_fightstick.getRawButtonPressed(3):
            self.Climbgal_L.getHomePosition()

        self.Climbgal_L.periodic()

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
        self.tm_which_climber = (
            self.tm_ClimberSelect.LEFT
            if self.sd_table.getBoolean("climber_LR_switch", False)
            else self.tm_ClimberSelect.RIGHT
        )
        selected_climber = self.Climbgal_L
        # TODO: Need to handle switching to the right climber

        # Make sure motors are stopped
        self.Climbgal_L.stop()

        # Determine the initial position of the ratchet switch
        if self.sd_table.getBoolean("ratchet_engage", False):
            selected_climber.__engageRatchet__()
        else:
            selected_climber.__disengageRatchet__()

        return super().testInit()

    def testPeriodic(self):
        self.Climbgal_L.testPeriodic()
        self.Climbgal_R.testPeriodic()

        self.sd_table.putNumber("climber_L_abs_enc", self.Climbgal_L.AbsEncoderVal())
        self.sd_table.putNumber("climber_R_abs_enc", self.Climbgal_R.AbsEncoderVal())
        # Puts encoder value onto the Network Table

        # Left Thumbstick
        joystickAxis = self.goodStick.getLeftY()
        self.Climbgal_L.motorDirect(joystickAxis)
        # Right Thumbstick
        joystickAxis = self.goodStick.getRightY()
        self.Climbgal_R.motorDirect(joystickAxis)

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

        # if self.pxn_fightstick.getRawButtonPressed(4):
        #     self.Climbgal.findHomePosition()

        # if self.pxn_fightstick.getRawButtonPressed(2):
        #     self.Climbgal.stop()

        # Below this point, the code is servicing the controls on the Test Mode screen of the custom dashboard

        # Determine which climber is selected
        which_climber = (
            self.tm_ClimberSelect.LEFT
            if self.sd_table.getBoolean("climber_LR_switch", False)
            else self.tm_ClimberSelect.RIGHT
        )

        # If the operator has switched the climber, stop the current climber
        if which_climber != self.tm_which_climber:
            pass
            # TODO: There should be code here for stopping the current climber when switching to the other one
            self.tm_which_climber = which_climber

        # Grab a reference to the selected climber
        selected_climber = self.Climbgal_L
        # TODO: Need to handle switching to the other climber

        # Handle the ratchet
        if self.sd_table.getBoolean("ratchet_engage", False):
            selected_climber.__engageRatchet__()
        else:
            selected_climber.__disengageRatchet__()

        # Display the absolute encoder value
        # TODO: Need to monitor the abs enc value and pass it to network tables

        # Monitor the joystick and turn them into motor commands


if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)
