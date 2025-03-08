import wpilib
from wpilib import Joystick
from subsystems import Climbguy
import wpimath
import rev



class ReefscapeRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.pxn_fightstick = wpilib.Joystick(0)
        
       
        
       
        
        # Initializing Servos
        self.andyMark = wpilib.Servo(0)
        self.andyMark.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )
        self.andyMark1 = wpilib.Servo(1)
        self.andyMark1.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )


        # Initializing Motor
        self.ClimberMotorLeft = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.ClimberMotorLeft.set(0.0)
        # self.ClimberMotorLeft.setExpiration(3.0)
        # self.ClimberMotorLeft.setSafetyEnabled(False)
        ClimbMotorLeftConfig = rev.SparkBaseConfig()
        ClimbMotorLeftConfig.inverted(True)
        self.ClimberMotorLeft.configure(ClimbMotorLeftConfig, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        # Initializing Other Motor
        self.ClimberMotorRight = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)
        self.ClimberMotorRight.set(0.0)
        # self.ClimberMotorRight.setExpiration(3.0)
        # self.ClimberMotorRight.setSafetyEnabled(False)
        ClimbMotorRightConfig = rev.SparkBaseConfig()
        ClimbMotorRightConfig.inverted(True)
        self.ClimberMotorRight.configure(ClimbMotorRightConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        # Initializing girly pop climbgal
        self.Climbgal = Climbguy.climb_mechanism(self.andyMark, self.andyMark1, self.relative_encoder, self.ClimberMotorLeft, self.ClimberMotorRight )

    def teleopInit(self):
       pass

    def teleopPeriodic(self):

    
    # Calling the methods made in Climbguy.py   
        if self.pxn_fightstick.getRawButtonPressed(1):
           self.Climbgal.climb()

        if self.pxn_fightstick.getRawButtonPressed(2):
           self.Climbgal.reset()
           
    # This stuff is for testing the motor 
           
        # if self.pxn_fightstick.getRawButtonPressed(3):
        #     self.Climbgal.__incrementRatchet__()
        
        # if self.pxn_fightstick.getRawButtonPressed(4): 
        #     self.Climbgal.__decrementRatchet__()
        


        self.Climbgal.periodic()
        
       
    def teleopExit(self):
        pass

    
    
    
    
    
    def testInit(self) -> None:
        
        # Testing the encoder 
        self.encodingTimer.restart()
        
        return super().testInit()
    
    def testPeriodic(self) -> None:
        
        # More testing of the encoder
        if self.encodingTimer.advanceIfElapsed(1):
            encodingVariable = self.encoderDude.get()
            print(f"output: {encodingVariable:0.3f}")
        
        if self.pxn_fightstick.getRawButtonPressed(5):
            self.ClimberMotorLeft.set(0.08)

        if self.pxn_fightstick.getRawButtonPressed(6):        
            self.ClimberMotorLeft.set(-0.08)

        if self.ClimberMotorLeft.get() >= 0.01:
            if self.encoderDude.get() >= 0.33:
                self.ClimberMotorLeft.set(0.0)

        if self.ClimberMotorLeft.get() <= -0.01:
            if self.encoderDude.get() <= 0.08:
                self.ClimberMotorLeft.set(0.0)
            
        if self.pxn_fightstick.getRawButtonPressed(1):
            self.ClimberMotorLeft.set(0.08)
        if self.pxn_fightstick.getRawButtonReleased(1):
            self.ClimberMotorLeft.set(0.0)
        
        return super().testPeriodic()


if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)

