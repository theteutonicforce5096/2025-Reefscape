import commands2
from commands2.sysid import SysIdRoutine
from commands2.cmd import print_

from subsystems.swerve_drive_constants import SwerveDriveConstants
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.intake import Intake

from pathplannerlib.auto import AutoBuilder, PathConstraints

from phoenix6 import SignalLogger

from wpilib import DriverStation
from wpimath.geometry import Transform2d, Pose2d

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # self.elevator = Elevator(40)
        # self.wrist = Wrist(50)
        # self.intake = Intake(51)

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)
        self.controller_2 = commands2.button.CommandXboxController(1)
        
        # Create max speed variables
        self.max_linear_speed = SwerveDriveConstants.max_linear_speed
        self.max_angular_rate = SwerveDriveConstants.max_angular_rate
    
    def configure_button_bindings_auto(self):
        # Reset the pose of the robot
        self.drivetrain.reset_pose(self.drivetrain.get_starting_position())  

        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            target_pose = self.drivetrain.get_state().pose.transformBy(Transform2d(Pose2d(), Pose2d(-0.25, 0, 0)))
        else: 
            target_pose = self.drivetrain.get_state().pose.transformBy(Transform2d(Pose2d(), Pose2d(0.25, 0, 0)))

        AutoBuilder.pathfindToPose(
            target_pose,
            PathConstraints(0.5, 1.0, 1.5, 3),
            0.0,
        ).schedule()

    def configure_button_bindings_teleop(self):   
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        
        # Reset slew rate limiters for controlling acceleration
        self.drivetrain.reset_slew_rate_limiters()

        # # Reset elevator setpoint
        # self.elevator.reset_setpoint()
        # self.wrist.reset_setpoint()
        
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
        #TODO: The use of DeferredCommand above looks fishy.
        # Reading the description of DeferredCommand, it looks like it's intended for use by
        # "lower priority" activities to execute as time allows. I would not consider drivetrain
        # processing to be a low priority task - quite the opposite, actually.
        # But more concerningly, is the part that says the isFinished() method of a DefaultCommand
        # should never return true. I don't think a DeferredCommand meets that requirement.
        # If indeed a DeferredCommand.isFinished() returns true at some point, then the next time
        # the robot needs to run a deferred command, it may have to create a new instance of
        # the DeferredCommand object. If this is occuring every 20ms, then it would have a significant
        # impact on performance.
        
        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )
        
        # # Set button binding for aligning to left side of reef face
        # self.controller.x().onTrue(
        #     commands2.DeferredCommand(
        #         lambda: self.drivetrain.get_reef_alignment_command(
        #             PathConstraints(
        #                 self.max_linear_speed * 0.10, 
        #                 self.max_linear_speed * 1, 
        #                 self.max_angular_rate * 0.10, 
        #                 self.max_angular_rate * 1
        #             ),
        #             0.0,
        #             self.controller,
        #             False
        #         ),
        #         self.drivetrain
        #     )
        # )

        # # Set button binding for aligning to right side of reef face
        # self.controller.b().onTrue(
        #     commands2.DeferredCommand(
        #         lambda: self.drivetrain.get_reef_alignment_command(
        #             PathConstraints(
        #                 self.max_linear_speed * 0.20, 
        #                 self.max_linear_speed * 0.60, 
        #                 self.max_angular_rate * 0.20, 
        #                 self.max_angular_rate * 0.60
        #             ),
        #             0.0,
        #             self.controller,
        #             True
        #         ),
        #         self.drivetrain
        #     )
        # )
        
        # self.controller_2.povUp().whileTrue(
        #     self.elevator.run(lambda: self.elevator.raise_setpoint())
        # )

        # self.controller_2.povDown().whileTrue(
        #     self.elevator.run(lambda: self.elevator.lower_setpoint())
        # )

        # self.controller_2.povRight().onTrue(
        #     self.elevator.runOnce(lambda: self.elevator.set_setpoint(100))
        # )

        # self.controller_2.povLeft().onTrue(
        #     self.elevator.runOnce(lambda: self.elevator.set_setpoint(0))
        # )

        # # self.controller_2.povLeft().onTrue(
        # #     self.elevator.runOnce(
        # #         lambda: print_(f"{self.elevator.encoder.getPosition()}, {self.elevator.setpoint}").schedule()
        # #     )
        # # )

        # self.controller_2.y().whileTrue(
        #     self.wrist.run(lambda: self.wrist.raise_setpoint())
        # )

        # self.controller_2.a().whileTrue(
        #     self.wrist.run(lambda: self.wrist.lower_setpoint())
        # )

        # self.intake.setDefaultCommand(
        #     self.intake.runOnce(lambda: self.intake.stop())
        # )

        # self.controller_2.b().onTrue(
        #     self.intake.runOnce(lambda: self.intake.intake())
        # )

        # self.controller_2.x().onTrue(
        #     self.intake.runOnce(lambda: self.intake.release())
        # )
    
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
