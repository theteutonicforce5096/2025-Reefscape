from commands2 import Subsystem
import phoenix6

class motor(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)
        phoenix6.CANBus.get_status()