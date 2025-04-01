import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath.units
from wpilib import Timer
import rev
from networktables import NetworkTables
from rev import ClosedLoopConfig
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class climb_mechanism:
    GEARRATIO = 155.6
    CURRENT_LIMIT_WEAK = 4  # Amps
    CURRENT_LIMIT_STRONG = 12  # Amps
    RATCHET_MOVE_TIME = 1.25  # seconds
    RATCHET_TIMER_AMOUNT = 1  # seconds
    MAX_ACCEL = 0.14  # encoder units per second^2
    MAX_SPEED = 0.25  # encoder units per second
    # MAX_POSITION = -0.45  # encoder
    # MIN_POSITION = -0.02  # encoder
    # ^^^^^ Used for Motor encoder- not absolute

    def __init__(self, servo_PWM_ID: int, motor_CAN_ID: int, dashboard_prefix: str, TARGET_POSITION_LIFT: float, TARGET_POSITION_ARMED: float):
        self.nt_rel_enc_value = dashboard_prefix + "_rel_enc"
        self.nt_abs_enc_value = dashboard_prefix + "_abs_enc"
        self.nt_motor_cmd_value = dashboard_prefix + "_motor_cmd"

        # Initializing Everything!!!

        # Build a motor config structure
        # Give is persistence with the object because we will use it
        # at different points in the climber's operation.
        self.motor_config = rev.SparkBaseConfig()
        self.motor_config.inverted(True)
        self.motor_config.encoder.positionConversionFactor(1.0 / self.GEARRATIO)
        self.motor_config.smartCurrentLimit(self.CURRENT_LIMIT_WEAK)

        # Now initialize and configure the motor
        self.ClimberMotor = rev.SparkMax(
            motor_CAN_ID, rev.SparkMax.MotorType.kBrushless
        )
        self.ClimberMotor.configure(
            self.motor_config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )
        # Initialize Servos
        self.andyMark = wpilib.Servo(servo_PWM_ID)
        self.andyMark.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )

        # Initializing TARGET POSITIONS
        self.TARGET_POSITION_LIFT = TARGET_POSITION_LIFT
        self.TARGET_POSITION_ARMED = TARGET_POSITION_ARMED

        # Initializing PID
        constraints = TrapezoidProfile.Constraints(
            maxVelocity=self.MAX_SPEED, maxAcceleration=self.MAX_ACCEL
        )
        self.PID = ProfiledPIDController(5, 0, 0, constraints=constraints)

        # Initalizing Encoder - We are using absolute
        self.motor_encoder = self.ClimberMotor.getEncoder()
        self.absolute_encoder = self.ClimberMotor.getAbsoluteEncoder()
        self.last_encoder_val = self.absolute_encoder.getPosition()
        self.absolute_enc_offset = 0.0

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
        current_encoder_val = self.absolute_encoder.getPosition()
        if (current_encoder_val - self.last_encoder_val) < -0.5:
            # we flipped!!!!!!
            self.absolute_enc_offset += 1.0

        elif (current_encoder_val - self.last_encoder_val) > 0.5:
            # we flipped!!!!!!
            self.absolute_enc_offset -= 1.0

        self.last_encoder_val = current_encoder_val

        return (current_encoder_val + self.absolute_enc_offset)

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
        self.andyMark.setPosition(disengange_position)

    def __engageRatchet__(self, engage_position=0.40):
        #  ratchet off!!!!!11!111
        self.andyMark.setPosition(engage_position)

    def climb(self):
        self.PID.setGoal(self.TARGET_POSITION_LIFT)

        # Negative value is the climb direction while positive is the reset.

    def reset(self):
        # if you dont know what reset is, go back to middle school
        self.__disengageRatchet__()
        self.ratchet_timer.restart()

    def teleopInit(self):
        self.last_encoder_val = self.absolute_encoder.getPosition()
        self.absolute_enc_offset = 0.0

        # Resetting PID
        self.__disengageRatchet__()
        self.PID.reset(self.getAbsoluteEncoderPosition())
        self.PID.setGoal(self.getAbsoluteEncoderPosition())
        self.motor_config.smartCurrentLimit(self.CURRENT_LIMIT_STRONG)
        self.ClimberMotor.configure(
            self.motor_config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )

    def periodic(self):
        self.sd_table.putNumber(
            self.nt_abs_enc_value, self.getAbsoluteEncoderPosition()
        )
        self.sd_table.putNumber(self.nt_rel_enc_value, self.getMotorEncoderPosition())

        # CLIMBING #
        current_location = self.getAbsoluteEncoderPosition()
        Motor_cmd = self.PID.calculate(current_location)
        Motor_cmd = min(Motor_cmd, 0.5)
        Motor_cmd = max(Motor_cmd, -0.5)
        # ^^^ if the number that the motor command is reading is higher than 0.1,
        # it will change it to 0.1 and if it is lower it will keep it there. This is a speed limit btw

        if (
            self.getAbsoluteEncoderPosition() > self.TARGET_POSITION_ARMED
            or self.getAbsoluteEncoderPosition() < self.TARGET_POSITION_LIFT
        ):
            self.ClimberMotor.set(0.0)
        self.ClimberMotor.set(Motor_cmd)
        self.sd_table.putNumber(self.nt_motor_cmd_value, Motor_cmd)

        #     # RESET #

        if self.ratchet_timer.isRunning():
            if self.ratchet_timer.hasElapsed(self.RATCHET_TIMER_AMOUNT):
                self.ratchet_timer.stop()
                self.PID.setGoal(self.TARGET_POSITION_ARMED)

    def motorDirect(self, motorSpeed: float):
        if not self.home_finding_mode:
            motor_cmd = motorSpeed / 10
            self.ClimberMotor.set(motor_cmd)
            self.sd_table.putNumber(self.nt_motor_cmd_value, motor_cmd)

    #### HOME POSITION STUFF ####

    # def findHomePosition(self):
    #     print("setting home")
    #     self.absolute_encoder

    # Reduce motor torque so we don't break anything!
    # self.motor_config.smartCurrentLimit(self.CURRENT_LIMIT_WEAK)
    # self.ClimberMotor.configure(
    #     self.motor_config,
    #     rev.SparkMax.ResetMode.kNoResetSafeParameters,
    #     rev.SparkMax.PersistMode.kNoPersistParameters
    # )

    # # Get the ratchet out of the way
    # self.__disengageRatchet__()
    # self.ratchet_timer.restart()

    # self.home_timer.stop()
    # self.LastEncoderValue = self.motor_encoder.getPosition()
    # self.home_finding_mode = True

    def testInit(self):
        self.last_encoder_val = self.absolute_encoder.getPosition()
        self.absolute_enc_offset = 0.0
        self.motor_config.smartCurrentLimit(self.CURRENT_LIMIT_WEAK)
        self.ClimberMotor.configure(
            self.motor_config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )

    def testPeriodic(self):
        self.sd_table.putNumber(
            self.nt_abs_enc_value, self.getAbsoluteEncoderPosition()
        )
        self.sd_table.putNumber(self.nt_rel_enc_value, self.getMotorEncoderPosition())

    ### AUTO HOME FINDING CODE BECAUSE I DON'T WANT TO LOSE IT ### > <
    #                                                               ^

    # finding home position - rest of the code

    # if self.home_finding_mode:

    #     current_encoder_value = self.motor_encoder.getPosition()

    #     # This if statement takes care of waiting until the ratchet gets out of the way
    #     if self.ratchet_timer.isRunning():
    #         if self.ratchet_timer.hasElapsed(self.RATCHET_MOVE_TIME):
    #             self.ratchet_timer.stop()
    #             print("ratchet timer elapsed")
    #             self.ClimberMotor.set(0.08)
    #             self.home_timer.restart()

    #     # This if statement looks for the climber to reach the end of its travel
    #     if self.home_timer.isRunning():
    #         if self.home_timer.advanceIfElapsed(1.0):
    #             if abs(current_encoder_value - self.LastEncoderValue) <= 0.001:
    #                 # Reached the end of travel
    #                 self.ClimberMotor.set(0.0)
    #                 self.home_timer.stop()
    #                 print("Found home")
    #                 self.motor_encoder.setPosition(0.0)
    #                 self.home_finding_mode = False

    # #   ^^^ If the motor is still running and the encoder hasn't changed,
    # #   ^^^ then we must be at the physical stop of the mechanism :DDD

    # # ^^^ We set the last encoder value to be equal to what it is equal to currently
    # # ^^^ This is home position

    # self.LastEncoderValue = current_encoder_value
