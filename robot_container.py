import commands2
from commands2.sysid import SysIdRoutine

from subsystems.swerve_drive_constants import SwerveDriveConstants

from phoenix6 import swerve, SignalLogger

from wpimath.units import rotationsToRadians
from wpimath.filter import SlewRateLimiter

from math import copysign

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)

        # Max speeds
        self.max_linear_speed = 5.63 # Max linear speed in meters per second
        self.max_angular_rate = rotationsToRadians(2.43) # Max angular velocity in radians per second 

        # Requests for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.drive = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
            .with_deadband(self.max_linear_speed * 0.01)
            .with_rotational_deadband(self.max_angular_rate * 0.01)
            .with_desaturate_wheel_speeds(True)
        )

        # Slew rate limiters for limiting robot acceleration
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed * 2, -self.max_linear_speed * 2)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed * 2, -self.max_linear_speed * 2)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate * 2, -self.max_angular_rate * 2)

    def configure_button_bindings_teleop(self):
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()

        self.drivetrain.tare_everything()

        limit_direction = lambda x: min(x, 0.20) if x >= 0 else max(x, -0.20)

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self.drive.with_velocity_x(
                        self.straight_speed_limiter.calculate(
                            (limit_direction(
                                -(self.controller.getLeftY() * abs(self.controller.getLeftY()))
                            ) * self.max_linear_speed)
                        )
                    ) 
                    .with_velocity_y(
                        self.strafe_speed_limiter.calculate(
                            (limit_direction(
                                -(self.controller.getLeftX() * abs(self.controller.getLeftX()))
                            ) * self.max_linear_speed)
                        )
                    )
                    .with_rotational_rate(
                        self.rotation_speed_limiter.calculate(
                            (limit_direction(
                                -(self.controller.getRightX() * abs(self.controller.getRightX()))
                            ) * self.max_angular_rate)
                        )
                    )
                )
            )
        )
        
        # Reset the field centric heading using left bumper, right bumper, and A button.
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))
    
    def configure_button_bindings_test(self):
        # Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        self.drivetrain.set_sys_id_routine()
    
        self.controller.leftBumper().onTrue(commands2.cmd.runOnce(SignalLogger.start))
        self.controller.rightBumper().onTrue(commands2.cmd.runOnce(SignalLogger.stop))

        self.controller.y().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.controller.a().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        self.controller.b().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        self.controller.x().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
