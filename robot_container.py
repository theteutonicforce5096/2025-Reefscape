import commands2
from commands2.sysid import SysIdRoutine
from commands2.cmd import print_

from subsystems.swerve_drive_constants import SwerveDriveConstants
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from pathplannerlib.auto import AutoBuilder, PathConstraints
from pathplannerlib.path import PathPlannerPath, GoalEndState

from phoenix6 import SignalLogger

from wpilib import DriverStation
from wpimath.geometry import Transform2d, Pose2d, Rotation2d

class RobotContainer:
    def __init__(self):
        self.elevator = Elevator(40)
        self.wrist = Wrist(50)
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)
        self.elevator_controller = commands2.button.CommandJoystick(1)
        
        # Create max speed variables
        self.max_linear_speed = SwerveDriveConstants.max_linear_speed
        self.max_angular_rate = SwerveDriveConstants.max_angular_rate
    
    def configure_button_bindings_auto(self):
        # Reset the pose of the robot
        starting_pose = self.drivetrain.get_starting_position()
        self.drivetrain.reset_pose(starting_pose)  

        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            target_pose = self.drivetrain.get_state().pose.transformBy(Transform2d(Pose2d(), Pose2d(-0.5, 0, 0)))
        else: 
            target_pose = self.drivetrain.get_state().pose.transformBy(Transform2d(Pose2d(), Pose2d(-0.5, 0, 0)))

        AutoBuilder.pathfindToPose(
            target_pose,
            PathConstraints(0.5, 1.0, 0.0, 0.0),
            0.0
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
            self.drivetrain.get_operator_drive_command(
                lambda: self.controller.getLeftTriggerAxis() > 0.1,
                lambda: self.controller.getRightTriggerAxis() > 0.1,
                lambda: self.controller.getLeftY(),
                lambda: self.controller.getLeftX(),
                lambda: self.controller.getRightX()
            ) 
        )
        
        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.controller.povUp().whileTrue(
            self.elevator.run(lambda: self.elevator.raise_setpoint_small())
        )
        
        self.controller.povDown().whileTrue(
            self.elevator.run(lambda: self.elevator.lower_setpoint_small())
        )
        
        self.elevator_controller.button(5).onTrue(
            self.elevator.runOnce(lambda: self.elevator.set_setpoint(0))
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
        
        self.elevator_controller.button(3).onTrue(
            self.elevator.runOnce(lambda: self.elevator.set_setpoint(24.78))
        )
        
        self.elevator_controller.button(4).onTrue(
            self.elevator.runOnce(lambda: self.elevator.set_setpoint(57))
        )
        
        self.elevator_controller.button(6).onTrue(
            self.elevator.runOnce(lambda: self.elevator.set_setpoint(95))
        )

        self.controller.x().onTrue(
            self.elevator.runOnce(
                lambda: print(self.wrist.encoder.getPosition(), self.wrist.setpoint)
            )
        )
             
        self.controller.y().whileTrue(
            self.wrist.run(lambda: self.wrist.set_setpoint(31))
        )
        
        self.controller.a().whileTrue(
            self.wrist.run(lambda: self.wrist.set_setpoint(0))
        )
        #Conflicting controls: Pressing 'a' also activated the intake/outtake motors
        
        self.controller.rightTrigger().onTrue(
            self.wrist.runOnce(lambda: self.wrist.find_home())
        )
        
        self.controller.leftTrigger().onTrue(
            self.wrist.runOnce(lambda: self.wrist.set_setpoint(15))
        )

        self.controller.rightBumper().onTrue(
            self.wrist.runOnce(lambda: self.wrist.raise_setpoint())
        )

        self.controller.leftBumper().onTrue(
            self.wrist.runOnce(lambda: self.wrist.lower_setpoint())
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
