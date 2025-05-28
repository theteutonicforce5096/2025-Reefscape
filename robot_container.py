import commands2

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

class RobotContainer:
    def __init__(self):
        self.elevator = Elevator(40)
        self.wrist = Wrist(50)

        self.controller = commands2.button.CommandXboxController(0)
        self.elevator_controller = commands2.button.CommandJoystick(1)
        
    def configure_button_bindings_teleop(self):
        self.wrist.setDefaultCommand(
            self.wrist.run(lambda: self.wrist.spin_motor(0))
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
        # 1'1" =]
        
        self.elevator_controller.button(3).onTrue(
            self.elevator.runOnce(lambda: self.elevator.set_setpoint(24.78))
        )
        # 2'7.875" =3
        
        self.elevator_controller.button(4).onTrue(
            self.elevator.runOnce(lambda: self.elevator.set_setpoint(57))
        )
        # 3'11.625" :OOO
        
        self.elevator_controller.button(6).onTrue(
            self.elevator.runOnce(lambda: self.elevator.set_setpoint(95))
        )
        # 6' =]]]]]]

        # self.controller.povRight().onTrue(
        #     self.elevator.run(lambda: self.elevator.raise_setpoint())
        # )

        # self.controller.povLeft().onTrue(
        #     self.elevator.run(lambda: self.elevator.lower_setpoint())
        # )
        
        # self.controller.povUp().whileTrue(
        #     self.elevator.run(lambda: self.elevator.spin_motor(.15))
        # )
        
        # self.controller.povDown().whileTrue(
        #     self.elevator.run(lambda: self.elevator.spin_motor(-.05))
        # )

        # self.controller.povRight().whileTrue(
        #     self.elevator.run(lambda: self.elevator.spin_motor(.05))
        # )

        # self.controller.povLeft().whileTrue(
        #     self.elevator.run(lambda: self.elevator.set_setpoint(25))
        # )

        self.controller.x().onTrue(
            self.elevator.runOnce(
                lambda: print(self.elevator.encoder.getPosition(), self.elevator.setpoint)
            )
        )
             
        self.controller.y().whileTrue(
            self.wrist.run(lambda: self.wrist.spin_motor(0.1))
        )
        
        self.controller.a().whileTrue(
            self.wrist.run(lambda: self.wrist.spin_motor(-0.1))
        )
    
    def configure_button_bindings_test(self):
        pass
    