import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from subsystems.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.units import rotationsToRadians

class RobotContainer:
    def __init__(self):
        # Desired top speed at 12 volts
        self._max_speed = TunerConstants.speed_at_12_volts 

        # Max angular velocity in rotations per second 
        self._max_angular_rate = rotationsToRadians(1)

        # Setting up bindings for controlling swerve drivezww
        self._drive = swerve.requests.FieldCentric(
            deadband = self._max_speed * 0.1,
            rotational_deadband = self._max_angular_rate * 0.1, 
            drive_request_type = swerve.SwerveModule.DriveRequestType.VELOCITY,
            steer_request_type = swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO,
            desaturate_wheel_speeds = True,
            forward_perspective = swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE
        )

        self._logger = Telemetry()

        self._controller = commands2.button.CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

    def configure_button_bindings_teleop(self):
        self.drivetrain.register_telemetry(lambda state: self._logger.telemeterize(state))

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        self._controller.getLeftY() * self._max_speed
                    )
                    .with_velocity_y(
                        self._controller.getLeftX() * self._max_speed
                    )
                    .with_rotational_rate(
                        self._controller.getRightX() * self._max_angular_rate
                    )
                )
            )
        )

        # Reset the field centric heading using left bumper, right bumper, and a button.
        (self._controller.leftBumper() & self._controller.rightBumper() & self._controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))
    
    def configure_button_bindings_test(self):
        # Note that each routine should be run exactly once in a single log.
        (self._controller.leftTrigger() & self._controller.leftBumper()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._controller.leftTrigger() & self._controller.rightBumper()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._controller.rightTrigger() & self._controller.leftBumper()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._controller.rightTrigger() & self._controller.rightBumper()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )
