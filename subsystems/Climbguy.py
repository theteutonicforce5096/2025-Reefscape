import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath
import wpimath.units
import robot
from wpilib import PWMSparkMax
from wpilib import Timer
import rev

class climb_mechanism:

    GEARRATIO = 155.6

    def __init__(self, left_servo:wpilib.Servo, right_servo:wpilib.Servo, encoderDude:wpilib.DutyCycleEncoder, left_motor:rev.SparkMax, right_motor:rev.SparkMax):
        
        
        # Figure out Home position by itself
        self.HomePosition = 0

        # Initializing Everything!!!
        self.andyMark = left_servo
        # self.andyMark1 = right_servo
        # self.encoderDude = encoderDude
        self.ClimberMotorLeft = left_motor
        self.MotorEncoder = self.ClimberMotorLeft.getEncoder()
        # self.ClimberMotorRight = right_motor
        
        
        self.__engageRatchet__()

       
        
        
        # Initialize timers
        self.climb_timer = wpilib.Timer()
        self.ratchet_timer = wpilib.Timer()
        self.motor_reset_timer = wpilib.Timer()
        self.print_timer = wpilib.Timer()
        self.print_timer.restart()


        # Initialize Encoder
        self.relative_encoder = self.ClimberMotorLeft.getEncoder()

    def __getRawEncoderVal__(self) -> float:
        return self.MotorEncoder.getPosition() 

    def __getArmPosition__(self):
        self.EncoderVal = self.__getRawEncoderVal__() 
        self.ArmPosition = self.EncoderVal / self.GEARRATIO - self.HomePosition
        
       
         
    def __disengageRatchet__(self):
        #  ratchet on!!!!11!!
        print("disengage ratchet")
        self.andyMark.setPosition(0.25)
       

    def __engageRatchet__(self):
        #  ratchet off!!!!!11!111
        print("engage ratchet")
        self.andyMark.setPosition(0.4)
       
    
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
        self.ClimberMotorLeft.set(-0.12)
        self.ClimberMotorRight.set(-0.12)
     
        
        
      
            
    def reset(self):
        # if you dont know what reset is, go back to ,middle school
        self.__disengageRatchet__()
        self.ratchet_timer.restart()


    def periodic(self):
        
        # CLIMBING #
        if self.ClimberMotorLeft.get() <= -0.01:
                if self.relative_encoder.get() <= 0.08:
                    self.ClimberMotorLeft.set(0.0)
                    
        # if self.ClimberMotorLeft.get() >= 0.01:
        #     if self.encoderDude.get() >= 0.33:
        #         self.ClimberMotorLeft.set(0.0)
                    print("It has successfully climbed!!")

        

       
       # RESET #
       
        if self.ratchet_timer.isRunning():
            if self.ratchet_timer.hasElapsed(2):
                self.ratchet_timer.stop()
              
                
       
        if self.ClimberMotorLeft.get() >= 0.01:
                if self.relative_encoder.get() >= 0.33:
                    self.ClimberMotorLeft.set(0.0)
        # if self.ClimberMotorLeft.get() <= -0.01:
                if self.encoderDude.get() <= 0.08:
        #         self.ClimberMotorLeft.set(0.0)
                    self.__engageRatchet__()
                    print("IT HAS SUCCESSFULLY RESET!!!!")



        
           
            


        



            
        
        
