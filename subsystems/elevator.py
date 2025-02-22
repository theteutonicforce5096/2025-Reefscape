from commands2 import Subsystem
import rev

class Elevator(Subsystem):
    def __init__(self, CAN_ID):
        Subsystem.__init__(self)

        self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
        motor_config = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20) # 20-40 recommended
            .inverted(False)
        )
        
        self.motor.configure(
            motor_config, 
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters
        )
                
    def spin_motor(self, percent):
        self.motor.set(percent)
        