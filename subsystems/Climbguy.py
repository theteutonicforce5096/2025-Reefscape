import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath
#TODO: No need to import wpimath.
import wpimath.units
import robot
#TODO: No need to import robot.
from wpilib import PWMSparkMax
#TODO No need to import PWMSparkMax - this was only used before when the motors were connected to the PWM ports of the roboRIO; now we're using CAN bus.
from wpilib import Timer
import rev
from networktables import NetworkTables

#TODO: Recommend you change the name of this file. By convention, the .py file is usually named the same as the class it contains.
class climb_mechanism:

    GEARRATIO = 155.6

    # The following constants are the names of the controls/indicators on the custom tashboard
    # used for low-level control during Test Mode operation
    TM_ENCODER_INDICATOR = 'encoder_value'
    TM_ARM_POSITION_INDICATOR = 'arm_position'

    def __init__(self, left_servo:wpilib.Servo, right_servo:wpilib.Servo, encoderDude:wpilib.DutyCycleEncoder, left_motor:rev.SparkMax, right_motor:rev.SparkMax):
        #TODO: We no longer need encoderDude because the encoder is no longer a separate device, but rather is a "sub-device" of our motor controller.
        #TODO: We no longer need two motor objects. A single climb_mechanism object only has one motor in it. At the robot.py level, we will instantiate a 2nd climb_mechanism object which will use the other motor.
        #TODO: Similarly, we no longer need two servos.
        
        # Figure out Home position by itself
        #TODO: We're not actually figuring out the home position here, we're just defining the variable.
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
        #TODO: We no longer need climb_timer. We will rely on the encoder to tell us when to stop climbing.
        self.ratchet_timer = wpilib.Timer()
        self.motor_reset_timer = wpilib.Timer()
        #TODO: We no longer need motor_reset_time. We will rely on the encoder to tell us when we're back in the arm position.
        self.print_timer = wpilib.Timer()
        self.print_timer.restart()
        #TODO: We no longer need print_timer. We were just using that in our first attempts to see if the motor encoder actually works.


        # Initialize Encoder
        self.relative_encoder = self.ClimberMotorLeft.getEncoder()
        #TODO: We no longer need self.relative_encoder. It's been replaced by self.MotorEncoder which is initialized just above.

        # Set up access to the network tables
        # - needed for communication with custom dashboard in Test mode
        NetworkTables.initialize(server='roborio-5096-frc.local')
        self.sd_table = NetworkTables.getTable('SmartDashboard')


    def __getRawEncoderVal__(self) -> float:
        return self.MotorEncoder.getPosition() 

    def __getArmPosition__(self):
        #TODO: This method needs to return something to the caller.
        self.EncoderVal = self.__getRawEncoderVal__() 
        self.ArmPosition = self.EncoderVal / self.GEARRATIO - self.HomePosition
        
    #TODO: Here, we will need a function that starts the process of finding the home position.
    # The funtion will just get things started by commanding motor to raise the climb arm at a slow rate.
    # The rest of the work will have to be done in the periodic function:
    #    - Keep watching the motor encoder to see if the value is changing.
    #    - Once it sees that the encoder value stops changing,
    #         - Stop the motor
    #         - Save the current encoder value as the new HomePosition.

         
    def __disengageRatchet__(self):
        #  ratchet on!!!!11!!
        print("disengage ratchet")
        self.andyMark.setPosition(0.25)
        #TODO: A good practice is to define the disengage position (0.25) as a constant right at the top of the class, and then refer to those constants throughout the body of the code.
       

    def __engageRatchet__(self):
        #  ratchet off!!!!!11!111
        print("engage ratchet")
        self.andyMark.setPosition(0.4)
       #TODO: A good practice is to define the engage position (0.40) as a constant right at the top of the class, and then refer to those constants throughout the body of the code.
    
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
        
        # Update the custom dashboard with current values
        self.sd_table.putNumber(self.TM_ENCODER_INDICATOR, self.__getRawEncoderVal__())
        self.sd_table.putNumber(self.TM_ARM_POSITION_INDICATOR, self.__getArmPosition__())

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



        
           
            


        



            
        
        
