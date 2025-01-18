from commands2 import Subsystem
import phoenix6

class Motor(Subsystem):
    def __init__(self, id):
        Subsystem.__init__(self)
        self.motor = phoenix6.hardware.TalonFX(id)

    def set_speed(self, percent):
        self.motor.set_control(phoenix6.controls.DutyCycleOut(percent))