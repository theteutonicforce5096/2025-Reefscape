import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from subsystems.tuner_constants import TunerConstants
from phoenix6 import swerve

from telemetry import Telemetry
from wpilib import DriverStation

from wpimath.units import rotationsToRadians
from wpimath.geometry import Rotation2d, Pose2d

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = TunerConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)

        # Initialize logger
        self.logger = Telemetry()
    
        # Custom variables
        self.max_speed = TunerConstants.speed_at_12_volts # Desired top speed at 12 volts
        self.max_angular_rate = rotationsToRadians(1) # Max angular velocity in rotations per second 

        # Setting up bindings for controlling swerve drive
        self.drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self.max_speed * 0.1)
            .with_rotational_deadband(self.max_angular_rate * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
            .with_desaturate_wheel_speeds(True)
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
        )

    def configure_button_bindings_teleop(self):
        # Register telemetry
        self.drivetrain.register_telemetry(lambda state: self.logger.telemeterize(state))
        
        # Set forward perspective for field oriented drive
        alliance_color = DriverStation.getAlliance()
        if alliance_color is not None:
            if alliance_color == DriverStation.Alliance.kBlue:
                # Blue alliance sees forward as 0 degrees (toward red alliance wall)
                self.drivetrain.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            else:
                # Red alliance sees forward as 180 degrees (toward blue alliance wall)
                self.drivetrain.set_operator_perspective_forward(Rotation2d.fromDegrees(180))  

        # Set initial pose for robot
        # Should be done in autonomous by PathPlanner for Competition
        # 5 meters right, 5 meters up, 0 radians rotation relative to blue alliance
        # where origin is at the bottom left of blue alliance
        self.drivetrain.reset_pose(Pose2d(5, 5, 0))

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self.drive.with_velocity_x(
                        self.controller.getLeftY() * self.max_speed
                    )
                    .with_velocity_y(
                        self.controller.getLeftX() * self.max_speed
                    )
                    .with_rotational_rate(
                        self.controller.getRightX() * self.max_angular_rate
                    )
                )
            )
        )

        # Reset the field centric heading using left bumper, right bumper, and A button.
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))
    
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
