import commands2

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

class RobotContainer:
    def __init__(self):
        self.elevator = Elevator(40)
        self.wrist = Wrist(50)

        self.controller = commands2.button.CommandXboxController(0)
        
    def configure_button_bindings_teleop(self):
        self.elevator.setDefaultCommand(
            self.elevator.run(lambda: self.elevator.spin_motor(0))
        )
        
        self.wrist.setDefaultCommand(
            self.wrist.run(lambda: self.wrist.spin_motor(0))
        )
        
        self.controller.povUp().whileTrue(
            self.elevator.run(lambda: self.elevator.spin_motor(0.2))
        )
        
        self.controller.povDown().whileTrue(
            self.elevator.run(lambda: self.elevator.spin_motor(-0.1))
        )
        
        self.controller.x().onTrue(
            self.elevator.runOnce(
                lambda: print(self.elevator.encoder.getPosition())
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
    