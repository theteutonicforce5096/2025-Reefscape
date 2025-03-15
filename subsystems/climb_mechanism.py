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
       
        

        # Initializing Everything!!!
        self.andyMark = left_servo
        self.ClimberMotorLeft = left_motor
        self.MotorEncoder = self.ClimberMotorLeft.getEncoder()
        
        
        self.__engageRatchet__()

       
        self.HomePosition = 0
        
        # Initialize timers
        self.ratchet_timer = wpilib.Timer()
        self.print_timer = wpilib.Timer()


    
        # Set up access to the network tables
        # - needed for communication with custom dashboard in Test mode
        NetworkTables.initialize(server='roborio-5096-frc.local')
        self.sd_table = NetworkTables.getTable('SmartDashboard')


    def getRawEncoderVal(self) -> float:
        return self.MotorEncoder.getPosition() 

    def getArmPosition(self):
        #This is all good we jsut need the home position defined 
        self.EncoderVal = self.__getRawEncoderVal__() 
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
        self.sd_table.putNumber(self.TM_ENCODER_INDICATOR, self.__getRawEncoderVal__())
        self.sd_table.putNumber(self.TM_ARM_POSITION_INDICATOR, self.__getArmPosition__())

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






#### HOME POSITION STUFF ####

    def findHomePosition(self):
        self.ClimberMotorLeft.set(0.1)
        self.LastEncoderValue = self.MotorEncoder.getPosition()
    


    def testPeriodic(self) -> None:

        self.CurrentEncoderValue = self.MotorEncoder.getPosition()

        self.CurrentEncoderValue - self.LastEncoderValue 
        if (self.CurrentEncoderValue - self.LastEncoderValue) <= 0:
            self.HomePosition = self.MotorEncoder.getPosition()
            self.ClimberMotorLeft.set(0)

        self.LastEncoderValue = self.CurrentEncoderValue


        
           
            


        



            
        
        
