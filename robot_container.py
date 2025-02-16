import commands2
from commands2.sysid import SysIdRoutine

from subsystems.swerve_drive_constants import SwerveDriveConstants

from pathplannerlib.auto import PathConstraints

from phoenix6 import SignalLogger

from math import pi

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)

    def configure_button_bindings_auto(self):
        # Set the starting pose of the robot
        starting_pose = self.drivetrain.get_starting_position()
        self.drivetrain.reset_pose(starting_pose)    

    def configure_button_bindings_teleop(self):
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()

        self.drivetrain.reset_slew_rate_limiters()
        
        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self.drivetrain.get_operator_drive_request(
                    self.controller.getHID().getLeftTriggerAxis() > 0.1,
                    self.controller.getHID().getRightTriggerAxis() > 0.1,
                    self.controller.getLeftY(),
                    self.controller.getLeftX(),
                    self.controller.getRightX()
                )
            )
        )
        
        # Reset the field centric heading using left bumper, right bumper, and A button.
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )
        
        self.controller.x().onTrue(
            self.drivetrain.get_reef_alignment_command(
                PathConstraints(1.0, 3.0, 1 * pi, 3 * pi),
                0.0,
                self.controller,
                False
            )
        )

        self.controller.b().onTrue(
            self.drivetrain.get_reef_alignment_command(
                PathConstraints(1.0, 3.0, 1 * pi, 3 * pi),
                0.0,
                self.controller,
                True
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
