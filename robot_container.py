import commands2
from commands2.sysid import SysIdRoutine

from subsystems.swerve_drive_constants import SwerveDriveConstants
from subsystems.elevator import Elevator

from pathplannerlib.auto import PathConstraints

from phoenix6 import SignalLogger

from math import pi

from commands2.cmd import print_ #for testing pid values

class RobotContainer:
    def __init__(self):
        # Initialize hardware
        # self.drivetrain = SwerveDriveConstants.create_drivetrain()
        self.elevator = Elevator(1, 50)

        # # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # # Max speed variables
        # self.max_linear_speed = SwerveDriveConstants.max_linear_speed
        # self.max_angular_rate = SwerveDriveConstants.max_angular_rate

    def configure_button_bindings_teleop(self):
        
        self.controller.a().whileTrue(
            commands2.RepeatCommand(
                self.elevator.runOnce(lambda: self.elevator.run_pid(.5))
                )
            )
        
        self.controller.x().whileTrue(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(-.1))
        )
        
        self.controller.b().whileTrue(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(.1))
        )
        
        # self.controller.x().whileTrue(
        #     commands2.DeferredCommand(
        #         lambda: self.elevator.spin_motor(
        #             (self.elevator.get_encoder_value()/5) #+ self.elevator.feedforward.calculate())/10 #help pls
        #         )
        #     )
        # )
        
        (self.controller.a()).onFalse(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(0))
        )
        
        (self.controller.b()).onFalse(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(0))
        )
        
        (self.controller.x()).onFalse(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(0))
        )
        
        
        self.controller.rightTrigger().whileTrue(
            self.elevator.run(lambda: self.elevator.print_encoder_position())  
        )   
        
        
        
        
        # starting_pose = self.drivetrain.get_starting_position()
        # self.drivetrain.reset_pose(starting_pose)    
            
        # # Set the forward perspective of the robot for field oriented driving
        # self.drivetrain.set_forward_perspective()

        # self.drivetrain.reset_slew_rate_limiters()
        
        # # Reset the field centric heading using left bumper, right bumper, and A button.
        # (self.controller.leftBumper() & self.controller.rightBumper() & self.controller.a()).onTrue(
        #     self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        # )
        
        # self.controller.x().onTrue(
        #     self.drivetrain.runOnce(
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
        #         ).schedule()
        #     )
        # )

        # self.controller.b().onTrue(
        #     self.drivetrain.runOnce(
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
        #         ).schedule()
        #     )
        # )
    
    def configure_button_bindings_test(self):
    #     # Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
    #     self.drivetrain.set_sys_id_routine()
    
    #     self.controller.leftBumper().onTrue(commands2.cmd.runOnce(SignalLogger.start))
    #     self.controller.rightBumper().onTrue(commands2.cmd.runOnce(SignalLogger.stop))

    #     self.controller.y().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward))
    #     self.controller.a().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
    #     self.controller.b().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
    #     self.controller.x().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
    
        self.controller.povUp().onTrue(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(.05))
        )
        
        self.controller.povDown().onTrue(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(-.05))
        )
        
        self.controller.povRight().onTrue(
            self.elevator.runOnce(lambda: self.elevator.spin_test())
        )
        
        self.controller.rightTrigger().whileTrue(
            self.elevator.run(lambda: self.elevator.print_encoder_position())  
        )   

        self.controller.povLeft().onTrue(
            self.elevator.runOnce(lambda: self.elevator.spin_motor(0))
        )
        
        # self.controller.povDown().onFalse(
        #     self.elevator.runOnce(lambda: self.elevator.spin_motor(0))
        # )