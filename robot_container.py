import commands2
from commands2.sysid import SysIdRoutine

from phoenix6 import swerve

from wpimath.units import rotationsToRadians
from wpimath.filter import SlewRateLimiter

from subsystems.swerve_drive_constants import SwerveDriveConstants

from telemetry import Telemetry

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)

        # Initialize logger
        self.logger = Telemetry()
    
        # Custom variables
        self.max_linear_speed = 5.63 # Max linear speed in meters per second
        self.max_angular_rate = rotationsToRadians(2.43) # Max angular velocity in radians per second 

        # Requests for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.drive = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.01)
            .with_rotational_deadband(self.max_angular_rate * 0.01)
            .with_desaturate_wheel_speeds(True)
        )

        self.brake = swerve.requests.SwerveDriveBrake()

        # Slew rate limiters for limiting robot acceleration
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed, -self.max_linear_speed)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed, -self.max_linear_speed)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate, -self.max_angular_rate)

    def configure_button_bindings_teleop(self):
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            commands2.ConditionalCommand(
                self.drivetrain.apply_request(
                    lambda: (
                        self.drive.with_velocity_x(
                            self.straight_speed_limiter.calculate(
                                (self.controller.getLeftY() ** 2) * self.max_linear_speed
                            )
                        )
                        .with_velocity_y(
                            self.strafe_speed_limiter.calculate(
                                -(self.controller.getLeftX() ** 2) * self.max_linear_speed
                            )
                        )
                        .with_rotational_rate(
                            self.rotation_speed_limiter.calculate(
                                -(self.controller.getRightX() ** 2) * self.max_angular_rate
                            )
                        )
                    )
                ),
                self.drivetrain.apply_request(lambda: self.brake),
                # Fix this condition later. Not sure how to compare states.
                lambda: True if self.drivetrain.get_state_copy().speeds == 0 else False
            )
        )
        
        # Reset the field centric heading using left bumper, right bumper, and A button.
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))
        
        # Register telemetry
        self.drivetrain.register_telemetry(lambda state: self.logger.telemeterize(state))
    
    def configure_button_bindings_test(self):
        # Note that each routine should be run exactly once in a single log.
        (self.controller.leftTrigger() & self.controller.leftBumper()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self.controller.leftTrigger() & self.controller.rightBumper()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self.controller.rightTrigger() & self.controller.leftBumper()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self.controller.rightTrigger() & self.controller.rightBumper()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )
