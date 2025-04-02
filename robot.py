import wpilib
# from subsystems.drivetrain import SwerveDrive
from subsystems.LED_controller import LED_controller
from wpilib import Joystick

class ReefscapeRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Set brownout voltage
        wpilib.RobotController.setBrownoutVoltage(6.3)
        self.pxn_fightstick = wpilib.Joystick(2)

        
     
        # Initialize LEDs
        self.lights = LED_controller()
        
        

      

        # Initialize timers
        self.timer = wpilib.Timer()


    def teleopInit(self):
        
      

        # Calling to set the alliance color 
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib._wpilib.DriverStation.Alliance.kRed:
            self.lights.set_alliance_color(True) 
        else: 
            self.lights.set_alliance_color(False)
        
        
            
        
       
    def teleopPeriodic(self):
       
        if self.pxn_fightstick.getRawButtonPressed(1):
            self.lights.LED_CP
            # Ingest

        if self.pxn_fightstick.getRawButtonPressed(2):
            self.lights.LED_CE
            # Expel

        if self.pxn_fightstick.getRawButtonPressed(5):
            self.lights.LED_L1
            # Level 1

        if self.pxn_fightstick.getRawButtonPressed(3):
            self.lights.LED_L2
            # Level 2

        if self.pxn_fightstick.getRawButtonPressed(4):
            self.lights.LED_L3
            # Level 3

        if self.pxn_fightstick.getRawButtonPressed(6):
            self.lights.LED_L4
            # Level 4

        if self.pxn_fightstick.getRawButtonPressed(9):
            self.lights.LED_CLIMB
            # Climb

    def teleopExit(self):
       
        pass

if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)