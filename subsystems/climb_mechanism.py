import wpilib
from wpilib import Servo
from wpilib import Joystick
import wpimath.units
from wpilib import Timer
import rev
from networktables import NetworkTables

class climb_mechanism:

    GEARRATIO = 155.6

    # The following constants are the names of the controls/indicators on the custom tashboard
    # used for low-level control during Test Mode operation
    TM_ENCODER_INDICATOR = 'encoder_value'
    TM_ARM_POSITION_INDICATOR = 'arm_position'

    def __init__(self, left_servo:wpilib.Servo, left_motor:rev.SparkMax):
        #TODO: Looks like we have to go back to using the external encoder... so need to add a parameter for robot.py to pass use the encoder object to use.

        # Initializing Everything!!!
        self.andyMark = left_servo
        self.ClimberMotorLeft = left_motor
        self.MotorEncoder = self.ClimberMotorLeft.getEncoder()
        
        
        self.__engageRatchet__()

       
        self.HomePosition = 0
        #TODO: Rather than auto-finding the home position, we will need to figure it out and pass it in during instantiation.
        
        # Initialize timers
        self.ratchet_timer = wpilib.Timer()
        self.print_timer = wpilib.Timer()
        self.home_timer = wpilib.Timer()


    
        # Set up access to the network tables
        # - needed for communication with custom dashboard in Test mode
        NetworkTables.initialize(server='roborio-5096-frc.local')
        self.sd_table = NetworkTables.getTable('SmartDashboard')


    def getRawEncoderVal(self) -> float:
        #TODO: Change the name of this function to make it clear that it gets the motor's encoder value
        return self.MotorEncoder.getPosition() 

    #TODO: Make a function that returns the absolute encoder's value

    def getArmPosition(self):
        #This is all good we jsut need the home position defined 
        self.EncoderVal = self.getRawEncoderVal() 
        self.ArmPosition = self.EncoderVal / self.GEARRATIO - self.HomePosition
        print(self.ArmPosition)
        
    
    def stop(self):
        #stops motor
        self.ClimberMotorLeft.set(0)

         
    def __disengageRatchet__(self, disengange_position = 0.25):
        #  ratchet on!!!!11!!
        print("disengage ratchet")
        self.andyMark.setPosition(disengange_position)

       

    def __engageRatchet__(self, engage_position = 0.40):
        #  ratchet off!!!!!11!111
        print("engage ratchet")
        self.andyMark.setPosition(engage_position)
       
    
    


    def climb(self):
       #Negative value is the climb direction while positive is the reset. 
        self.ClimberMotorLeft.set(-0.12)
       
            
    def reset(self):
        # if you dont know what reset is, go back to ,middle school
        self.__disengageRatchet__()
        self.ratchet_timer.restart()



    def periodic(self):
        
        # Update the custom dashboard with current values
        self.sd_table.putNumber(self.TM_ENCODER_INDICATOR, self.getRawEncoderVal())
        self.sd_table.putNumber(self.TM_ARM_POSITION_INDICATOR, self.getArmPosition())

        # CLIMBING #
        if self.ClimberMotorLeft.get() <= -0.01:
                if self.MotorEncoder.get() <= 0.08:
                    self.ClimberMotorLeft.set(0.0)
                    print("It has successfully climbed!!")

        

       
       # RESET #
       
        if self.ratchet_timer.isRunning():
            if self.ratchet_timer.hasElapsed(2):
                self.ratchet_timer.stop()
                self.ClimberMotorLeft.set(0.12)
              
                
        if self.ClimberMotorLeft.get() >= 0.01:
                if self.MotorEncoder.get() >= 0.33:
                    self.ClimberMotorLeft.set(0.0)
                    self.__engageRatchet__()
                    print("IT HAS SUCCESSFULLY RESET!!!!")


    #TODO: Make a function that allows Test Mode to command the motor to move




    #### HOME POSITION STUFF ####

    #TODO: Comment this home-finding code out for now. We may use it later, but for now, we will rely on the absolute encoder for positioning.

    def findHomePosition(self):
        self.__disengageRatchet__()
        self.ratchet_timer.restart()
        self.home_timer.stop()
        

       
        self.LastEncoderValue = self.MotorEncoder.getPosition()
    


    def testPeriodic(self):

        # finding home position - rest of the code
        
        current_encoder_value = self.MotorEncoder.getPosition()

        # This if statement takes care of waiting until the ratchet gets out of the way
        if self.ratchet_timer.isRunning():
            if self.ratchet_timer.hasElapsed(2):
                print("ratchet timer elapsed")
                self.ClimberMotorLeft.set(0.04)
                self.ratchet_timer.reset()

                self.home_timer.restart()
                self.LastEncoderValue = current_encoder_value


        if self.home_timer.isRunning():
            if self.home_timer.advanceIfElapsed(.25):
                print(f"{current_encoder_value:0.2f}")
                if current_encoder_value - self.LastEncoderValue <= 0.01:
                    self.ClimberMotorLeft.set(0.0)
                    blah = rev.SparkBaseConfig()
                    blah.smartCurrentLimit()
                    self.ClimberMotorLeft.configure(blah)
                    self.home_timer.stop()
                    self.HomePosition = current_encoder_value
                else:
                    self.LastEncoderValue = current_encoder_value
        #   ^^^ If the motor is still running and the encoder hasn't changed, 
        #   ^^^ then we must be at the physical stop of the mechanism :DDD
    
    
    
        
        # ^^^ We set the last encoder value to be equal to what it is equal to currently
        # ^^^ This is home position
            
            

        
           
            


        



            
        
        
