import commands2
from commands2.sysid import SysIdRoutine
from commands2.cmd import print_

from subsystems.swerve_drive_constants import SwerveDriveConstants

from phoenix6 import swerve, SignalLogger

from pathplannerlib.auto import AutoBuilder, PathConstraints

from wpimath.filter import SlewRateLimiter
from wpimath.units import rotationsToRadians
from math import pi

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = SwerveDriveConstants.create_drivetrain(0.832, 0.0, 0.1651)

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)

        # Create max speed variables
        self.max_linear_speed = 5.63 # Max linear speed in meters per second
        self.max_angular_rate = rotationsToRadians(2.43) # Max angular velocity in radians per second 

        # Create request for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.01)
            .with_rotational_deadband(self.max_angular_rate * 0.01)
            .with_desaturate_wheel_speeds(True)
        )

        # Create slew rate limiters for limiting robot acceleration
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed * 2, -self.max_linear_speed * 2)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed * 2, -self.max_linear_speed * 2)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate * 2, -self.max_angular_rate * 2)

    def configure_button_bindings_teleop(self):
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()

        # Set the starting pose of the robot in odometry and Limelight
        starting_pose = self.drivetrain.get_starting_position()
        self.drivetrain.reset_pose(starting_pose)    

        self.straight_speed_limiter.reset(self.drivetrain.get_state_copy().speeds.vx)
        self.strafe_speed_limiter.reset(self.drivetrain.get_state_copy().speeds.vy)
        self.rotation_speed_limiter.reset(self.drivetrain.get_state_copy().speeds.omega)

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self.field_centric_request.with_velocity_x(
                        self.straight_speed_limiter.calculate(
                            -(self.controller.getLeftY() * abs(self.controller.getLeftY())) * self.max_linear_speed
                        )
                    )
                    .with_velocity_y(
                        self.strafe_speed_limiter.calculate(
                            -(self.controller.getLeftX() * abs(self.controller.getLeftX())) * self.max_linear_speed
                        )
                    )
                    .with_rotational_rate(
                        self.rotation_speed_limiter.calculate(
                            -(self.controller.getRightX() * abs(self.controller.getRightX())) * self.max_angular_rate
                        )
                    )
                )
            )
        )
        
        # Reset the field centric heading using left bumper, right bumper, and A button.
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )
        
        self.controller.x().onTrue(
            lambda: (
                AutoBuilder.pathfindToPose(
                    target_pose,
                    PathConstraints(1.0, 3.0, 1 * pi, 3 * pi),
                    0.0
                ).until(
                    lambda: self.controller.getHID().getYButtonPressed()
                )
                if (target_pose := self.drivetrain.get_target_reef_pose(False)) != None
                else print_("Reef Tag Not Found.")
            )
        )

        self.controller.b().onTrue(
            lambda: (
                AutoBuilder.pathfindToPose(
                    target_pose,
                    PathConstraints(1.0, 3.0, 1 * pi, 3 * pi),
                    0.0
                ).until(
                    lambda: self.controller.getHID().getYButtonPressed()
                )
                if (target_pose := self.drivetrain.get_target_reef_pose(True)) != None
                else print_("Reef Tag Not Found.")
            )
        )
    
    def configure_button_bindings_test(self):
        # Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        self.drivetrain.set_sys_id_routine()
    
        self.controller.leftBumper().onTrue(commands2.cmd.runOnce(SignalLogger.start))
        self.controller.rightBumper().onTrue(commands2.cmd.runOnce(SignalLogger.stop))

        self.controller.y().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.controller.a().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        self.controller.b().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        self.controller.x().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
