import commands2
from commands2.sysid import SysIdRoutine

from subsystems.swerve_drive_constants import SwerveDriveConstants

from pathplannerlib.auto import PathConstraints

from phoenix6 import SignalLogger

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # Create max speed variables
        self.max_linear_speed = SwerveDriveConstants.max_linear_speed
        self.max_angular_rate = SwerveDriveConstants.max_angular_rate

    def configure_button_bindings_teleop(self):
        # Reset the pose of the robot
        self.drivetrain.reset_pose(self.drivetrain.get_starting_position())    
            
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        
        # Reset slew rate limiters for controlling acceleration
        self.drivetrain.reset_slew_rate_limiters()
        
        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            commands2.DeferredCommand(
                lambda: self.drivetrain.get_operator_drive_command(
                    self.controller.getLeftTriggerAxis() > 0.1,
                    self.controller.getRightTriggerAxis() > 0.1,
                    self.controller.getLeftY(),
                    self.controller.getLeftX(),
                    self.controller.getRightX()
                ),
                self.drivetrain
            )
        )
        
        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )
        
        # Set button binding for aligning to left side of reef face
        self.controller.x().onTrue(
            commands2.DeferredCommand(
                lambda: self.drivetrain.get_reef_alignment_command(
                    PathConstraints(
                        self.max_linear_speed * 0.10, 
                        self.max_linear_speed * 1, 
                        self.max_angular_rate * 0.10, 
                        self.max_angular_rate * 1
                    ),
                    0.0,
                    self.controller,
                    False
                ),
                self.drivetrain
            )
        )

        # Set button binding for aligning to right side of reef face
        self.controller.b().onTrue(
            commands2.DeferredCommand(
                lambda: self.drivetrain.get_reef_alignment_command(
                    PathConstraints(
                        self.max_linear_speed * 0.20, 
                        self.max_linear_speed * 0.60, 
                        self.max_angular_rate * 0.20, 
                        self.max_angular_rate * 0.60
                    ),
                    0.0,
                    self.controller,
                    True
                ),
                self.drivetrain
            )
        )
    
    def configure_button_bindings_test(self):
        # Set the SysId routine to run
        self.drivetrain.set_sys_id_routine()
    
        # Set button bindings for starting and stopping SignalLogger
        self.controller.leftBumper().onTrue(commands2.cmd.runOnce(SignalLogger.start))
        self.controller.rightBumper().onTrue(commands2.cmd.runOnce(SignalLogger.stop))

        # Set button bindings for performing various parts of SysID routine
        self.controller.y().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.controller.a().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        self.controller.b().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        self.controller.x().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
