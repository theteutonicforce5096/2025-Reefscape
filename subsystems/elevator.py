from commands2 import Subsystem
import rev
from commands2.cmd import print_
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ElevatorFeedforward
from networktables import NetworkTables

class Elevator(Subsystem):
    def __init__(self, CAN_ID):
        Subsystem.__init__(self)

        NetworkTables.initialize(server='roborio-5096-frc.local')
        self.sd_table = NetworkTables.getTable('SmartDashboard')
        
        self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        motor_config = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40) # 20-40 recommended
            .inverted(True)
        )

        self.motor.configure(
            motor_config, 
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters
        )
        
        self.pid_controller = ProfiledPIDController(1, 0, 0, TrapezoidProfile.Constraints(1.0, 1.0))
        self.feedforward = ElevatorFeedforward(0.2, 0, 0, 0)
                
        self.setpoint = 0.00

        self.encoder.setPosition(0.0)
                
    def run_pid(self):
        current_position = self.encoder.getPosition()
        output = self.pid_controller.calculate(current_position, self.setpoint)
        self.spin_motor(output)
        self.sd_table.putNumber(
            'elevator_motor', output
        )
        self.sd_table.putNumber(
            'elevator_rel_enc', current_position
        )
        self.sd_table.putNumber(
            'elevator_motor_setpoint', self.setpoint
        )
    
    def spin_motor(self, percent):
        self.motor.set(percent)
    
    def raise_setpoint(self):
        if self.setpoint < 8:
            self.setpoint += 1
        
    def lower_setpoint(self):
        if self.setpoint >= 1:
            self.setpoint -= 1