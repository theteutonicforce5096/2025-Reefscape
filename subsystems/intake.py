from commands2 import Subsystem

import rev

class  Intake(Subsystem):
    def __init__(self, CAN_ID):
        Subsystem.__init__(self)
        pass
        
    #     self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)

    #     motor_config = (
    #         rev.SparkMaxConfig()
    #         .voltageCompensation(12.0)
    #         .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    #         .smartCurrentLimit(20) # 20-40 recommended
    #         .inverted(True)
    #     )

    #     self.motor.configure(
    #         motor_config, 
    #         rev.SparkBase.ResetMode.kResetSafeParameters,
    #         rev.SparkBase.PersistMode.kNoPersistParameters
    #     )
    
    def intake(self):
        pass
    #     self.motor.set(0.1)

    def release(self):
        pass
    #     self.motor.set(-0.1)

    def stop(self):
        pass
    #     self.motor.set(0)