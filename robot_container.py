import commands2
from commands2.sysid import SysIdRoutine

from subsystems.swerve_drive_constants import SwerveDriveConstants

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)

    def configure_button_bindings_teleop(self):
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            self.drivetrain.set_robot_speed(
                0.01, 
                0.01,
                self.controller.getLeftY(),
                self.controller.getLeftX(),
                self.controller.getRightX()
            )
        )
        
        # Reset the field centric heading using left bumper, right bumper, and A button.
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))
    
    def configure_button_bindings_test(self):
        # Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        self.drivetrain.set_sys_id_routine()

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
