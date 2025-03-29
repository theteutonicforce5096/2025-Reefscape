from commands2 import Subsystem
from commands2.cmd import SequentialCommandGroup, WaitCommand
import rev

class Wrist(Subsystem):
    def __init__(self, CAN_ID):
        Subsystem.__init__(self)
        
        self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()
        
        self.motor_config = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20) # 20-40 recommended
            .inverted(False) # inverted is CL, regular is CCL
        )
        
        self.motor_config_find_home = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(5) # 20-40 recommended
            .inverted(False) # inverted is CL, regular is CCL
        )
        
        self._configure_motor(self.motor_config)
        
    def _configure_motor(self, motor_config):
        self.motor.configure(
            motor_config, 
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters
        )
        
    def find_home(self):
        self._configure_motor(self.motor_config_find_home)
        SequentialCommandGroup(
            self.runOnce(lambda: self.spin_motor(-0.1)),
            WaitCommand(0.1),
            self.run(
                lambda: self.spin_motor(-0.05)
            ).until(
                lambda: round(self.encoder.getVelocity(), 2) == 0
            ),
            self.runOnce(lambda: self.reset_encoder()),
            self.runOnce(lambda: self.spin_motor(0))
        ).schedule()
        
    def reset_encoder(self):
        self.encoder.setPosition(0) 

    def spin_motor(self, percent):
        self.motor.set(percent)       
        
    # def run_pid(self, goal):
    #     pos = self.get_encoder_value()
    #     speed = self.pid_controller.calculate(pos, goal)
    #     self.motor_speed = speed
    #     self.spin_motor(speed)
        
    # def get_encoder_value(self):
    #     return (self.encoder.getPosition())
    
    # def print_encoder_position(self):
    #     self.encoder_position = self.get_encoder_value() # encoder reads values from -.5 to .5
    #     print_(self.encoder_position).schedule()
        
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
        
    # def find_encoder_change(self):
    #     normal_distance = .3
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
    #     if status == 'stuck':
    #         self.motor.set(0)
        
    # the goal is to set the motor to a low voltage and check before and after pos to see if there is any resistance
    # def spin_test(self):
    #     self.configure_motor(False, 5)
    #     self.initial_pos = self.get_encoder_value()
    #     SequentialCommandGroup(
    #         WaitCommand(1), 
    #         self.runOnce(lambda: self.find_encoder_change())
    #     ).schedule()