import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath.units
from wpilib import Timer
import rev
from networktables import NetworkTables
from rev import ClosedLoopConfig



class climb_mechanism:
    GEARRATIO = 155.6
    CURRENT_LIMIT_WEAK = 4          # Amps
    CURRENT_LIMIT_STRONG = 20       # Amps
    RATCHET_MOVE_TIME = 1.25        # seconds
    TARGET_POSTION = -0.42          # encoder

    def __init__(
        self,
        left_servo: wpilib.Servo,
        motor_CAN_ID: int,
        # encoderDude: rev.SparkAbsoluteEncoder,
    ):
        # Initializing Everything!!!
        self.andyMark = left_servo

        # Build a motor config structure
        # Give is persistence with the object because we will use it
        # at different points in the climber's operation.
        self.motor_config = rev.SparkBaseConfig()
        self.motor_config.inverted(True)
        self.motor_config.encoder.positionConversionFactor(1.0 / self.GEARRATIO)
        self.motor_config.smartCurrentLimit(self.CURRENT_LIMIT_WEAK)

        # Now initialize and configure the motor
        self.ClimberMotor = rev.SparkMax(motor_CAN_ID, rev.SparkMax.MotorType.kBrushless)
        self.ClimberMotor.configure(
            self.motor_config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters
        )
        self.PID_Controller = self.ClimberMotor.getClosedLoopController()
        self.PID_Controller.setReference(-0.42, rev.SparkMax.ControlType.kPosition, 0)


        self.motor_encoder = self.ClimberMotor.getEncoder()
        self.absolute_encoder = self.ClimberMotor.getAbsoluteEncoder()

        self.__engageRatchet__()

        # Initialize timers
        self.ratchet_timer = wpilib.Timer()
        self.print_timer = wpilib.Timer()
        self.home_timer = wpilib.Timer()

        # Set up access to the network tables
        # - needed for communication with custom dashboard in Test mode
        NetworkTables.initialize(server="roborio-5096-frc.local")
        self.sd_table = NetworkTables.getTable("SmartDashboard")

        self.home_finding_mode = False

    def getMotorEncoderPosition(self) -> float:
        return self.motor_encoder.getPosition()

    def getAbsoluteEncoderPosition(self) -> float:
        return self.absolute_encoder.getPosition()

    # def getArmPosition(self):
    #     # This is all good we jsut need the home position defined
    #     self.EncoderVal = self.getRawMotorEncoderVal()
    #     self.ArmPosition = self.EncoderVal / self.GEARRATIO - self.HomePosition
    #     print(self.ArmPosition)

    def stop(self):
        # stops motor
        self.ClimberMotor.set(0)

    def __disengageRatchet__(self, disengange_position=0.25):
        #  ratchet on!!!!11!!
        # print("disengage ratchet")
        self.andyMark.setPosition(disengange_position)

    def __engageRatchet__(self, engage_position=0.40):
        #  ratchet off!!!!!11!111
        # print("engage ratchet")
        self.andyMark.setPosition(engage_position)

    def climb(self):
        # Negative value is the climb direction while positive is the reset.
        self.ClimberMotor.set(-0.05)
    def reset(self):
        # if you dont know what reset is, go back to middle school
        self.__disengageRatchet__()
        self.ratchet_timer.restart()

    def periodic(self):
        # Update the custom dashboard with current values
        # self.sd_table.putNumber(self.TM_ENCODER_INDICATOR, self.getMotorEncoderPosition())
        # self.sd_table.putNumber(self.TM_ARM_POSITION_INDICATOR, self.getArmPosition())

        # CLIMBING #
        if self.ClimberMotor.get() <= -0.01:
            if self.motor_encoder.getPosition() <= self.TARGET_POSTION:
                self.ClimberMotor.set(0.0)
                print("It has successfully climbed!!")

        # RESET #

        if self.ratchet_timer.isRunning():
            if self.ratchet_timer.hasElapsed(2):
                self.ratchet_timer.stop()
                self.ClimberMotor.set(0.05)

        if self.ClimberMotor.get() >= 0.01:
            if self.motor_encoder.getPosition() >= 0:
                self.ClimberMotor.set(0.0)
                self.__engageRatchet__()
                print("IT HAS SUCCESSFULLY RESET!!!!")

    def motorDirect(self, motorSpeed: float):
        if not self.home_finding_mode:
            self.ClimberMotor.set(motorSpeed / 10)

    #### HOME POSITION STUFF ####

    def findHomePosition(self):

        # Reduce motor torque so we don't break anything!
        self.motor_config.smartCurrentLimit(self.CURRENT_LIMIT_WEAK)
        self.ClimberMotor.configure(
            self.motor_config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters
        )

        # Get the ratchet out of the way
        self.__disengageRatchet__()
        self.ratchet_timer.restart()

        self.home_timer.stop()
        self.LastEncoderValue = self.motor_encoder.getPosition()
        self.home_finding_mode = True

    def testPeriodic(self):

        # finding home position - rest of the code

        if self.home_finding_mode:

            current_encoder_value = self.motor_encoder.getPosition()

            # This if statement takes care of waiting until the ratchet gets out of the way
            if self.ratchet_timer.isRunning():
                if self.ratchet_timer.hasElapsed(self.RATCHET_MOVE_TIME):
                    self.ratchet_timer.stop()
                    print("ratchet timer elapsed")
                    self.ClimberMotor.set(0.08)
                    self.home_timer.restart()

            # This if statement looks for the climber to reach the end of its travel
            if self.home_timer.isRunning():
                if self.home_timer.advanceIfElapsed(1.0):
                    if abs(current_encoder_value - self.LastEncoderValue) <= 0.001:
                        # Reached the end of travel
                        self.ClimberMotor.set(0.0)
                        self.home_timer.stop()
                        print("Found home")
                        self.motor_encoder.setPosition(0.0)
                        self.home_finding_mode = False
                        
                    # #   ^^^ If the motor is still running and the encoder hasn't changed,
                    # #   ^^^ then we must be at the physical stop of the mechanism :DDD

                    # # ^^^ We set the last encoder value to be equal to what it is equal to currently
                    # # ^^^ This is home position
            
            self.LastEncoderValue = current_encoder_value

