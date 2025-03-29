from commands2 import Subsystem
import rev
from commands2.cmd import print_
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ElevatorFeedforward

class Elevator(Subsystem):
    def __init__(self, CAN_ID):
        Subsystem.__init__(self)
        
        self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getAbsoluteEncoder()

        motor_config = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40) # 20-40 recommended
            .inverted(True)
        )
        
        motor_config.absoluteEncoder.inverted(True)
        motor_config.absoluteEncoder.zeroOffset(0.5736510)

        self.motor.configure(
            motor_config, 
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters
        )
        
        self.pid_controller = ProfiledPIDController(1, 0, 0, TrapezoidProfile.Constraints(1.0, 1.0))
        self.feedforward = ElevatorFeedforward(0.2, 0, 0, 0)
                
        self.setpoint = 0.00
        self.last_encoder_position = 0.00
                
    def run_pid(self):
        current_position = self.encoder.getPosition()
        output = self.pid_controller.calculate(current_position, self.setpoint)
        self.spin_motor(output)
    
    def spin_motor(self, percent):
        self.motor.set(percent)   