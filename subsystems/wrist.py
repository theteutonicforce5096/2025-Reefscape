from commands2 import Subsystem
from commands2 import SequentialCommandGroup, WaitCommand
import rev
from wpimath.controller import ProfiledPIDController, PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ArmFeedforward

class Wrist(Subsystem):
    def __init__(self, CAN_ID):
        pass
        # Subsystem.__init__(self)
        
        # self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
        # self.encoder = self.motor.getEncoder()

        # motor_config = (
        #     rev.SparkMaxConfig()
        #     .voltageCompensation(12.0)
        #     .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        #     .smartCurrentLimit(20) # 20-40 recommended
        #     .inverted(False)
        # )

        # self.motor.configure(
        #     motor_config, 
        #     rev.SparkBase.ResetMode.kResetSafeParameters,
        #     rev.SparkBase.PersistMode.kNoPersistParameters
        # )
        
        # self.pid_controller = PIDController(0.01, 0, 0)
        # self.feedforward = ArmFeedforward(0, 0, 0, 0)
        
        # self.encoder.setPosition(0.0)
        # self.setpoint = 0

    def reset_setpoint(self):
        pass
        # self.encoder.setPosition(0.0)
        # self.setpoint = 0

    def run_pid(self):
        pass
        # current_position = self.encoder.getPosition()
        # pid_output = self.pid_controller.calculate(current_position, self.setpoint)
        # feedforward = 0 # self.feedforward.calculate((self.encoder.getVelocity() * (1.067/120)) / 60)
        # output = pid_output + feedforward

        # self.spin_motor(output)
    
    def spin_motor(self, percent):
        pass
        # self.motor.set(percent)
    
    def raise_setpoint(self):
        pass
        # if self.setpoint < 25:
        #     self.setpoint += 1
        
    def lower_setpoint(self):
        pass
        # if self.setpoint >= 1:
        #     self.setpoint -= 1

    def set_setpoint(self, setpoint):
        pass
        # self.setpoint = setpoint