import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath
import wpimath.units
import robot
from wpilib import PWMSparkMax
from wpilib import Timer


class climb_mechanism:

    def __init__(self):
        
        
        # Initializing Servos
        self.andyMark = wpilib.Servo(0)
        self.andyMark1 = wpilib.Servo(2)
        
        # The andyMark expects pulse widths between 1000 µs and 2000 µs
        self.andyMark.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )
        self.andyMark1.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )
        
        
        self.ratchetPosition = 0.25
        # BETWEEN 40 AND 25 !!!

        # Initializing Motor
        self.neo_motor = wpilib.PWMSparkMax(1)
        self.neo_motor.set(0.0)
        self.neo_motor.setExpiration(3.0)
        self.neo_motor.setSafetyEnabled(False)
        
        # Initializing Other Motor
        self.neo_motor1 = wpilib.PWMSparkMax(3)
        self.neo_motor1.set(0.0)
        self.neo_motor1.setExpiration(3.0)
        self.neo_motor1.setSafetyEnabled(False)
        
        # Initialize timers
        self.climb_timer = wpilib.Timer()
        self.ratchet_timer = wpilib.Timer()
        self.motor_reset_timer = wpilib.Timer()
        




    def __disengageRatchet__(self):
        #  ratchet on!!!!11!!
        print("disengage ratchet")
        self.andyMark.setPosition(0.25)
        self.andyMark1.setPosition(0.25)

    def __engageRatchet__(self):
        #  ratchet off!!!!!11!111
        print("engage ratchet")
        self.andyMark.setPosition(0.4)
        self.andyMark1.setPosition(0.4)
        
    
    # TEST STUFF #
    
    # def __incrementRatchet__(self):
    #     Addguy = self.andyMark1.getPosition()
    #     Addguy += 0.02
    #     Addguy = min(Addguy,1.0)
    #     self.andyMark1.setPosition(Addguy)
    #     print(f"ratchetpos.: {Addguy:0.2f}")
        
        
    # def __decrementRatchet__(self):
    #     Subtractguy = self.andyMark1.getPosition()
    #     Subtractguy -= 0.02
    #     Subtractguy = max(Subtractguy,0)
    #     self.andyMark1.setPosition(Subtractguy)
    #     print(f"ratchetpos.: {Subtractguy:0.2f}")

    # def RM(self):
    #     # Testing the motor and running it to see if it works
    #     output = 0.2
    #     print(f"output: {output:0.2f}")
    #     self.neo_motor.set(output)
    
    # def SM(self):
    #     print("Stop motor")
    #     self.neo_motor.set(0.0)



    def climb(self):
        # when it activates the climb thing and turns the servo to latch in the 3d printed thing
        self.__engageRatchet__()
        self.neo_motor.set(.12)
        self.neo_motor1.set(.12)
        self.climb_timer.restart()
        
            
    def reset(self):
        # if you dont know what reset is, go back to ,middle school
        self.__disengageRatchet__()
        self.ratchet_timer.restart()


    def periodic(self):
        
        # CLIMBING #
        
        if self.climb_timer.isRunning() and self.climb_timer.hasElapsed(2):
            self.neo_motor.set(0)
            self.neo_motor1.set(0)
            self.climb_timer.stop()
            print("yippie")
                
       
       # RESET #
       
        if self.ratchet_timer.isRunning():
            if self.ratchet_timer.hasElapsed(2):
                self.ratchet_timer.stop()
                self.neo_motor.set(-.08)
                self.neo_motor1.set(-.08)
                self.motor_reset_timer.restart()
       
       
        if self.motor_reset_timer.isRunning():
            if self.motor_reset_timer.hasElapsed(2):
                self.neo_motor.set(0)
                self.neo_motor1.set(0)
                self.motor_reset_timer.stop()
                self.__engageRatchet__()
                print("IT HAS SUCCESSFULLY RESET!!!!")
        
            
        
        
