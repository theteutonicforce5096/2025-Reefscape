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

        # Initializing Left SERVO
        self.andyMark_L = wpilib.Servo(0)
        self.andyMark_L.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )

        # Initializing Right SERVO
        self.andyMark_R = wpilib.Servo(1)
        self.andyMark_R.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )


        # Initializing Left MOTOR
        # self.ClimberMotorLeft = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        # self.ClimberMotorLeft.set(0.0)

        # ClimberMotorLeftConfig = rev.SparkBaseConfig()
        # ClimberMotorLeftConfig.inverted(True)
        # self.ClimberMotorLeft.configure(
        #     ClimberMotorLeftConfig,
        #     rev.SparkMax.ResetMode.kNoResetSafeParameters,
        #     rev.SparkMax.PersistMode.kNoPersistParameters,
        # )

        # Initializing Right MOTOR
        # self.ClimberMotorRight = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)
        # self.ClimberMotorRight.set(0.0)

        # ClimberMotorRightConfig = rev.SparkBaseConfig()
        # ClimberMotorRightConfig.inverted(True)
        # self.ClimberMotorRight.configure(
        #     ClimberMotorRightConfig,
        #     rev.SparkMax.ResetMode.kNoResetSafeParameters,
        #     rev.SparkMax.PersistMode.kNoPersistParameters,
        # )


        # Initializing Left ENCODER
        # self.encoderDude_L = wpilib.DutyCycleEncoder(2)
        # self.encoderDude_L.setAssumedFrequency(wpimath.units.hertz(975.6))
        # self.encoderDude_L.setDutyCycleRange(min=1 / 1025, max=1024 / 1025)
        # self.encoderDude_L.setInverted(True)
        # self.encoderDude_L = self.ClimberMotorLeft.getAbsoluteEncoder()

        # Initializing Right ENCODER
        # self.encoderDude_R = wpilib.DutyCycleEncoder(3)
        # self.encoderDude_R.setAssumedFrequency(wpimath.units.hertz(975.6))
        # self.encoderDude_R.setDutyCycleRange(min=1 / 1025, max=1024 / 1025)
        # self.encoderDude_R.setInverted(True)
        # self.encoderDude_R = self.ClimberMotorRight.getAbsoluteEncoder()

       

        # Initializing girly pop climbgal left
        self.Climbgal_L = climb_mechanism.climb_mechanism(
            self.andyMark_L, 61
        )

        # Initializing girly pop climbgal right
        self.Climbgal_R = climb_mechanism.climb_mechanism(
            self.andyMark_R, 62
        )

        # Network tables are used for Test Mode
        NetworkTables.initialize(server="roborio-5096-frc.local")
        self.sd_table = NetworkTables.getTable("SmartDashboard")

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass
        # # Calling 'Get absolute encoder'
        # self.Climbgal_L.AbsEncoderVal()

        # # Calling the methods made in Climbguy.py

        # if self.pxn_fightstick.getRawButtonPressed(3):
        #     self.Climbgal_L.getHomePosition()

        # self.Climbgal_L.periodic()

    def teleopExit(self):
        pass

    ### TEST MODE STUFF ###

    def testInit(self):
        self.Climbgal_L.stop()
        self.Climbgal_R.stop()
        # Testing the encoder
        # self.encodingTimer.restart()

        # The following code has to do with servcing the controls on the Test Mode screen of the custom dashboard
        # Make sure motors are stopped

        return super().testInit()

    def testPeriodic(self):
        self.Climbgal_L.testPeriodic()
        self.Climbgal_R.testPeriodic()

        self.sd_table.putNumber("climber_L_abs_enc", self.Climbgal_L.getAbsoluteEncoderPosition())
        self.sd_table.putNumber("climber_R_abs_enc", self.Climbgal_R.getAbsoluteEncoderPosition())
        self.sd_table.putNumber("climber_L_rel_enc", self.Climbgal_L.getMotorEncoderPosition())
        self.sd_table.putNumber("climber_R_rel_enc", self.Climbgal_R.getMotorEncoderPosition())
        # Puts encoder value onto the Network Table

        # Left Thumbstick
        joystickAxis = self.goodStick.getLeftY()
        self.Climbgal_L.motorDirect(-joystickAxis)
        # Right Thumbstick
        joystickAxis = self.goodStick.getRightY()
        self.Climbgal_R.motorDirect(-joystickAxis)

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

        if self.goodStick.getXButtonPressed():
            self.Climbgal_L.findHomePosition()

        if self.goodStick.getBButtonPressed():
            self.Climbgal_R.findHomePosition()

if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)
