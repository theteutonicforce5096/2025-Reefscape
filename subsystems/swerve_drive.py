from commands2 import Subsystem
from commands2.sysid import SysIdRoutine

from phoenix6 import SignalLogger, swerve

from wpilib import DriverStation
from wpilib.shuffleboard import Shuffleboard
from wpilib import SendableChooser

from math import pi
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from wpimath.filter import SlewRateLimiter

from wpilib import Field2d

class SwerveDrive(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants, modules):
        """
        Constructs a swerve drivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type: Type of the drive motor
        :type drive_motor_type: type
        :param steer_motor_type: Type of the steer motor
        :type steer_motor_type: type
        :param encoder_type: Type of the azimuth encoder
        :type encoder_type: type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants: swerve.SwerveDrivetrainConstants
        :param modules: Constants for each specific module
        :type modules: list[swerve.SwerveModuleConstants]
        """

        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, modules)
        
        # Create Field2d Widget in Shuffleboard
        self.field = Field2d()
        Shuffleboard.getTab("Drivers").add(f"Field", self.field).withSize(3, 3)
        
        # Register telemetry function for updating pose
        self.register_telemetry(lambda state: self.update_pose(state))

        # Max speeds
        self.max_linear_speed = 5.63 # Max linear speed in meters per second
        self.max_angular_rate = rotationsToRadians(2.43) # Max angular velocity in radians per second 

        # Requests for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.drive = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_desaturate_wheel_speeds(True)
        )

        self.brake = swerve.requests.SwerveDriveBrake()

        # Slew rate limiters for limiting robot acceleration
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed, -self.max_linear_speed)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed, -self.max_linear_speed)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate, -self.max_angular_rate)

        # Swerve requests for SysId characterization
        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self.rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # SysId routine for characterizing drive.
        self.sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 4.0
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.translation_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing steer.
        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 7.0,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.steer_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing rotation. This is used for FieldCentricFacingAngle HeadingController.
        self.sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate = pi / 6,
                stepVoltage = 7.0,
            ),
            SysIdRoutine.Mechanism(
                # output is actually radians per second, but SysId only supports "volts"
                lambda output: self.set_control(self.rotation_characterization.with_rotational_rate(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine to test
        self.sys_id_routines = SendableChooser()
        self.sys_id_routines.setDefaultOption("Translation Routine", self.sys_id_routine_translation)
        self.sys_id_routines.addOption("Steer Routine", self.sys_id_routine_steer)
        self.sys_id_routines.setDefaultOption("Rotation Routine", self.sys_id_routine_rotation)

        Shuffleboard.getTab("SysId").add(f"Routines", self.sys_id_routines).withSize(2, 2)
        self.sys_id_routine_to_apply = self.sys_id_routines.getSelected()
    
    def set_robot_speed(self, translation_deadband_percent, rotation_deadband_percent, left_y, left_x, right_x):
        """
        Set the speed of the robot from controller inputs.

        :param translation_deadband_percent: Translation deadband in percent of max linear speed
        :type translation_deadband_percent: float
        :param rotation_deadband_percent: Rotation deadband in percent of max angular speed 
        :type rotation_deadband_percent: float
        :param left_y: Y value of the left side of the controller
        :type left_y: float
        :param left_x: X value of the left side of the controller
        :type left_x: float
        :param right_x: X value of the right side of the controller
        :type right_x: float
        """

        # Convert joystick inputs into robot speeds with slew rate limiter applied
        straight_speed = self.straight_speed_limiter.calculate((left_y ** 2) * self.max_linear_speed)
        strafe_speed = self.strafe_speed_limiter.calculate((left_x ** 2) * self.max_linear_speed)
        rotation_speed = self.rotation_speed_limiter.calculate((right_x ** 2) * self.max_angular_rate)

        translation_deadband = self.max_linear_speed * translation_deadband_percent
        rotation_deadband =  self.max_angular_rate * rotation_deadband_percent

        # Move robot if speeds are above deadband otherwise stop robot
        if abs(straight_speed) >= translation_deadband or abs(strafe_speed) >= translation_deadband or abs(rotation_speed) >= rotation_deadband:
            drivetrain_request = lambda: (
                self.drive.with_velocity_x(straight_speed)
                .with_velocity_y(strafe_speed)
                .with_rotational_rate(rotation_speed)
            )

            return self.run(lambda: self.set_control(drivetrain_request()))
        else:
            self.run(lambda: self.set_control(self.brake()))
    
    def set_forward_perspective(self):
        """
        Set forward perspective of the robot for field oriented drive.
        """

        alliance_color = DriverStation.getAlliance()
        if alliance_color is not None:
            if alliance_color == DriverStation.Alliance.kBlue:
                # Blue alliance sees forward as 0 degrees (toward red alliance wall)
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            else:
                # Red alliance sees forward as 180 degrees (toward blue alliance wall)
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))  

    def update_pose(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to Shuffleboard.
        https://www.chiefdelphi.com/t/fatal-python-error-segmentation-fault-when-using-register-telemetry-in-phoenix-6-swerve-drive-api/483721/4
        """
        self.field.setRobotPose(state.pose)

    def set_sys_id_routine(self):
        """
        Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        """
        self.sys_id_routine_to_apply = self.sys_id_routines.getSelected()

    def sys_id_quasistatic(self, direction):
        """
        Runs the SysId Quasistatic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction):
        """
        Runs the SysId Dynamic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.dynamic(direction)
