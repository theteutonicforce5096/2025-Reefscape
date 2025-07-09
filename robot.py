import wpilib
from subsystems.Coral_Manipulator import coral_manipulator
from networktables import NetworkTables


#from subsystems.drivetrain import SwerveDrive

class ReefscapeRobot(wpilib.TimedRobot):
    def robotInit(self):
       self.goodStick = wpilib.XboxController(1)
       self.pxn_fightstick = wpilib.Joystick(0)
       
       self.coral_manipulator = coral_manipulator() 
       
       #Configuring Timer
       self.intake_timer = wpilib.Timer()

    def teleopInit(self):
        pass
    

    def teleopPeriodic(self):
        if self.pxn_fightstick.getRawButtonPressed(1):
            print("Sanity Check")
            self.intake_timer.restart()
            self.coral_manipulator.intake()
        if self.intake_timer.hasElapsed(3):
            self.coral_manipulator.stop()
       
    def teleopExit(self):
        pass
      
if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)