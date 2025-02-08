import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath
import wpimath.units
import robot
from wpilib import PWMSparkMax


class climb_mechanism:

    def __init__(self):
        # Initializing Servo
        self.andyMark = wpilib.Servo(3)
        # The andyMark expects pulse widths between 1000 µs and 2000 µs
        self.andyMark.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )
        self.ratchetPosition = 0.05
        # BETWEEN 40 AND 25 !!!

        # Initializing Motor
        self.neo_motor = wpilib.PWMSparkMax(1)
        self.neo_motor.set(0.0)
        self.neo_motor.setExpiration(3.0)
        self.neo_motor.setSafetyEnabled(False)
        # Initialize timer

    def __disengageRatchet__(self):
        #  ratchet on!!!!11!!
        print("disengage ratchet")
        self.andyMark.setPosition(0.25)

    def __engageRatchet__(self):
        #  ratchet off!!!!!11!111
        print("engage ratchet")
        self.andyMark.setPosition(0.4)

    # def IR(self):
    #     # Testing the ratchet back and forth in increments
    #     self.ratchetPosition += 0.05
    #     self.ratchetPosition = min(self.ratchetPosition, 1.0)
    #     print(f"{self.ratchetPosition:0.2f}")

    #     self.andyMark.setPosition(self.ratchetPosition)

    # def DR(self):
    #     # Testing the ratchet back and forth in increments
    #     self.ratchetPosition -= 0.05
    #     self.ratchetPosition = max(self.ratchetPosition, 0)
    #     print(f"{self.ratchetPosition:0.2f}")

    #     self.andyMark.setPosition(self.ratchetPosition)

    def RM(self):
        # Testing the motor and running it to see if it works
        output = 0.2
        print(f"output: {output:0.2f}")
        self.neo_motor.set(output)
    
    def SM(self):
        print("Stop motor")
        self.neo_motor.set(0.0)

    def climb(self):
        # when it activates the climb thing and turns the servo to latch in the 3d printed thing
        pass

    def reset(self):
        # if you dont know what reset is, go back to ,middle school
        pass

    def periodic(self):
        if not self.neo_motor.isAlive():
            print("Not alive!")
        pass
