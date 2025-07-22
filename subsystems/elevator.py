from commands2 import Subsystem
from commands2.cmd import print_

import rev

from networktables import NetworkTables

from wpimath.controller import ProfiledPIDController, PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ElevatorFeedforward


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
        
        self.pid_controller = ProfiledPIDController(.1, 0, .001, TrapezoidProfile.Constraints(35, 16))
        self.feedforward = ElevatorFeedforward(0.2, 2, 0, 0)
                
        self.setpoint = 0

        self.encoder.setPosition(0.0)

    def reset_setpoint(self):
        pass
#         self.encoder.setPosition(0.0)
#         self.setpoint = 0

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

    def raise_setpoint_small(self):
        if self.setpoint < 95:   #smoked at top of 105, 100 ify
            self.setpoint += .5
        
    def lower_setpoint_small(self):
        if self.setpoint >= 1:
            self.setpoint -= .5

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
