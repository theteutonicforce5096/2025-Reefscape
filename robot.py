import wpilib
from subsystems.Coral_Manipulator import coral_manipulator

#from subsystems.drivetrain import SwerveDrive

class ReefscapeRobot(wpilib.TimedRobot):
    def robotInit(self):
       self.goodStick = wpilib.XboxController(1)
       self.coral_manipulator = coral_manipulator 

    def teleopInit(self):
        pass
    

    def teleopPeriodic(self):
        pass
       
    def teleopExit(self):
        pass
      
if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)