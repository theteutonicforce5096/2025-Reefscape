import wpilib
from wpilib import Joystick
from subsystems import climb_mechanism
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

        # Initializing Motor
        self.ClimberMotorLeft = rev.SparkMax(49, rev.SparkMax.MotorType.kBrushless)
        self.ClimberMotorLeft.set(0.0)
       
        ClimbMotorLeftConfig = rev.SparkBaseConfig()
        ClimbMotorLeftConfig.inverted(True)
        self.ClimberMotorLeft.configure(ClimbMotorLeftConfig, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        # Initializing girly pop climbgal
        self.Climbgal = climb_mechanism.climb_mechanism(self.andyMark, self.ClimberMotorLeft )

    def teleopInit(self):
       pass

    def teleopPeriodic(self):

    
    # Calling the methods made in Climbguy.py   
      
        if self.pxn_fightstick.getRawButtonPressed(3):
            self.Climbgal.__getHomePosition__()
        


        self.Climbgal.periodic()
        
       
    def teleopExit(self):
        pass

    
    
    
    
    
    def testInit(self):
        
        # Testing the encoder 
        # self.encodingTimer.restart()
        
        return super().testInit()
    
    def testPeriodic(self):
        
        if self.pxn_fightstick.getRawButtonPressed(4):
            self.Climbgal.findHomePosition()

        if self.pxn_fightstick.getRawButtonPressed(2):
            self.Climbgal.stop()
        
        self.Climbgal.testPeriodic()

if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)

