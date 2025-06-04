import wpilib
from subsystems.Coral_Manipulator import coral_manipulator
from networktables import NetworkTables


#from subsystems.drivetrain import SwerveDrive

class ReefscapeRobot(wpilib.TimedRobot):
    def robotInit(self):
       self.goodStick = wpilib.XboxController(1)
       self.pxn_fightstick = wpilib.Joystick(0)
       self.coral_manipulator = coral_manipulator 

    def teleopInit(self):
        pass
    

    def teleopPeriodic(self):
        if self.pxn_fightstick.getRawButtonPressed(1):
            self.coral_manipulator.intake()
       
    def teleopExit(self):
        pass
      
if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)