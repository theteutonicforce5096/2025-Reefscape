from commands2 import Subsystem, SequentialCommandGroup, WaitCommand
import rev
from wpilib import DutyCycleEncoder
from commands2.cmd import print_
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ElevatorFeedforward
import phoenix6

class Elevator(Subsystem):
    def __init__(self, CAN_ID, encoder_id):
        Subsystem.__init__(self)

        self.encoder = phoenix6.hardware.CANcoder(encoder_id, 'CANivore')  # accepts the canID
        # self.encoder = DutyCycleEncoder(encoder_id)
        
        self.pid_controller = ProfiledPIDController(1.5, 0, 0, TrapezoidProfile.Constraints(.5, .5)) #needs tuning
        self.pid_controller.enableContinuousInput(-.5, .5)
        self.feedforward = ElevatorFeedforward(0, 0, 0, 0) #find values latr
        
        
        self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
        motor_config = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20) # 20-40 recommended
            .inverted(False) # inverted is CL, regular is CCL
        )
        
        self.motor.configure(
            motor_config, 
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters
        )

    def configure_motor(self, inverted):
        motor_config = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20) # 20-40 recommended
            .inverted(inverted) # inverted is CL, regular is CCL
        )

        self.motor.configure(
            motor_config, 
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters
        )

    def spin_motor(self, percent):
        self.motor_speed = percent
        self.motor.set(percent)
        
    def run_pid(self, goal):
        pos = self.get_encoder_value()
        speed = self.pid_controller.calculate(pos, goal)
        self.motor_speed = speed
        self.spin_motor(speed)
        
    def get_encoder_value(self):
        return (self.encoder.get_absolute_position().value)
    
    def print_encoder_position(self):
        self.encoder_position = self.get_encoder_value() # encoder reads values from -.5 to .5
        print_(self.encoder_position).schedule()
        
    # def find_encoder_change(self):
    #     normal_distance = .6
    #     self.final_pos = self.get_encoder_value()
    #     if self.final_pos < self.initial_pos and self.motor_speed > 0:
    #         self.final_pos += 1
    #     if self.initial_pos < self.final_pos and self.motor_speed < 0:
    #         self.final_pos -= 1
    #     change_in_pos = self.final_pos - self.initial_pos
    #     status = 'movin'
    #     if normal_distance > change_in_pos > -normal_distance:
    #         status = 'stuck'
    #     print_((change_in_pos, status)).schedule()
        
    def find_encoder_change(self):
        normal_distance = .3
        self.final_pos = self.get_encoder_value()
        if self.final_pos < self.initial_pos and self.motor_speed > 0:
            self.final_pos += 1
        if self.initial_pos < self.final_pos and self.motor_speed < 0:
            self.final_pos -= 1
        change_in_pos = self.final_pos - self.initial_pos
        status = 'movin'
        if normal_distance > change_in_pos > -normal_distance:
            status = 'stuck'
        print_((change_in_pos, status)).schedule()
        if status == 'stuck':
            self.motor.set(0)
        
    # the goal is to set the motor to a low voltage and check before and after pos to see if there is any resistance
    def spin_test(self):
        self.initial_pos = self.get_encoder_value()
        SequentialCommandGroup(
            WaitCommand(1), 
            self.runOnce(lambda: self.find_encoder_change())
        ).schedule()