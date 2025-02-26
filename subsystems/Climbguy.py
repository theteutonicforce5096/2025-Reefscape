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

    def __init__(self, left_servo:wpilib.Servo, right_servo:wpilib.Servo, encoderDude:wpilib.DutyCycleEncoder, left_motor:rev.SparkMax, right_motor:rev.SparkMax):
        
        
        # Initializing Everything!!!
        self.andyMark = left_servo
        self.andyMark1 = right_servo
        self.encoderDude = encoderDude
        self.ClimberMotorLeft = left_motor
        self.ClimberMoterRight = right_motor
        
        # self.andyMark1 = wpilib.Servo(1)
        
        # Initializing andyMarks
        # The andyMark expects pulse widths between 1000 µs and 2000 µs
        # self.andyMark.setBounds(
        #     max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        # )
        # self.andyMark1.setBounds(
        #     max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        # )
        
        self.__engageRatchet__()

        # Initializing encoder
        # self.encoderDude = wpilib.DutyCycleEncoder(0)
        # self.encoderDude.setAssumedFrequency(wpimath.units.hertz (975.6))
        # self.encoderDude.setDutyCycleRange(min = 1/1025, max = 1024/1025)
        # self.encoderDude.setInverted(True)
       
        # BETWEEN 40 AND 25 !!
        # Arm in up position: 0.080
        # Arm in climb position: 0.33

        # # Initializing Motor
        # self.ClimberMotorLeft = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        # self.ClimberMotorLeft.set(0.0)
        # # self.ClimberMotorLeft.setExpiration(3.0)
        # # self.ClimberMotorLeft.setSafetyEnabled(False)
        # ClimbMotorLeftConfig = rev.SparkBaseConfig()
        # ClimbMotorLeftConfig.inverted(True)
        # self.ClimberMotorLeft.configure(ClimbMotorLeftConfig, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        

        
        # # Initializing Other Motor
        # self.ClimberMotorRight = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)
        # self.ClimberMotorRight.set(0.0)
        # # self.ClimberMotorRight.setExpiration(3.0)
        # # self.ClimberMotorRight.setSafetyEnabled(False)
        # self.config = rev.SparkBaseConfig()
        # self.config.follow(1)
        # self.ClimberMotorRight.configure(self.config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        
        
        
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
        self.ClimberMotorLeft.set(-0.12)
     
        
        
      
            
    def reset(self):
        # if you dont know what reset is, go back to ,middle school
        self.__disengageRatchet__()
        self.ratchet_timer.restart()


    def periodic(self):
        
        # CLIMBING #
        if self.ClimberMotorLeft.get() <= -0.01:
            if self.encoderDude.get() <= 0.08:
                self.ClimberMotorLeft.set(0.0)
        # if self.ClimberMotorLeft.get() >= 0.01:
        #     if self.encoderDude.get() >= 0.33:
        #         self.ClimberMotorLeft.set(0.0)
                print("It has successfully climbed!!")

        # if self.climb_timer.isRunning() and self.climb_timer.hasElapsed(2):
        #     self.ClimberMotorLeft.set(0)
        #     self.ClimberMotorRight.set(0)
        #     self.climb_timer.stop()
        #     print("yippie")
                
       
       # RESET #
       
        if self.ratchet_timer.isRunning():
            if self.ratchet_timer.hasElapsed(2):
                self.ratchet_timer.stop()
                self.ClimberMotorLeft.set(0.08)
                
       
        if self.ClimberMotorLeft.get() >= 0.01:
            if self.encoderDude.get() >= 0.33:
                self.ClimberMotorLeft.set(0.0)
        # if self.ClimberMotorLeft.get() <= -0.01:
        #     if self.encoderDude.get() <= 0.08:
        #         self.ClimberMotorLeft.set(0.0)
                self.__engageRatchet__()
                print("IT HAS SUCCESSFULLY RESET!!!!")

        
        
            
        
        
