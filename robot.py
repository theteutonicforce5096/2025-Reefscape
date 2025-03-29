from enum import Enum, auto
import wpilib
from wpilib import Joystick
from networktables import NetworkTables
from subsystems import climb_mechanism
import wpimath
import rev


class ReefscapeRobot(wpilib.TimedRobot):

    #TODO: Would be good to put a bunch of constants here that contain all the port and CAN ID definitions. It keeps all that information together and easy to find.
    def robotInit(self):
        self.pxn_fightstick = wpilib.Joystick(1)
        self.goodStick = wpilib.XboxController(0)

        # Initializing Left SERVO
        #TODO: Since the function of the climbers depends on this specific ratchet/servo combination, it would really be more appropriate to move servo initialization into the climber_mechanism class (in the same way we moved the motor config).
        self.andyMark_L = wpilib.Servo(0)
        self.andyMark_L.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )

        # Initializing Right SERVO
        self.andyMark_R = wpilib.Servo(1)
        self.andyMark_R.setBounds(
            max=2500, deadbandMax=1500, center=1500, deadbandMin=1500, min=500
        )

        # Initializing girly pop climbgal left
        self.Climbgal_L = climb_mechanism.climb_mechanism(
            self.andyMark_L, 61, "climber_L"
        )

        # Initializing girly pop climbgal right
        self.Climbgal_R = climb_mechanism.climb_mechanism(
            self.andyMark_R, 62, "climber_R"
        )

        # Network tables are used for Test Mode
        NetworkTables.initialize(server="roborio-5096-frc.local")
        self.sd_table = NetworkTables.getTable("SmartDashboard")

    def teleopInit(self):
        self.Climbgal_L.teleopInit()
        self.Climbgal_R.teleopInit()

    def teleopPeriodic(self):
        if self.pxn_fightstick.getRawButtonPressed(9):
            print("button 9")
            self.Climbgal_L.climb()
            self.Climbgal_R.climb()
        if self.pxn_fightstick.getRawButtonPressed(10):
            print("button 10")
            self.Climbgal_L.reset()
            self.Climbgal_R.reset()

        self.Climbgal_L.periodic()
        self.Climbgal_R.periodic()

    def teleopExit(self):
        pass

    ### TEST MODE STUFF ###

    def testInit(self):
        self.Climbgal_L.stop()
        self.Climbgal_R.stop()
        self.Climbgal_R.testInit()
        self.Climbgal_L.testInit()
        # Testing the encoder
        # self.encodingTimer.restart()

        # The following code has to do with servcing the controls on the Test Mode screen of the custom dashboard
        # Make sure motors are stopped

        return super().testInit()

    def testPeriodic(self):
        self.Climbgal_L.testPeriodic()
        self.Climbgal_R.testPeriodic()

        # Left Thumbstick
        joystickAxis_L = -self.goodStick.getLeftY()
        self.Climbgal_L.motorDirect(joystickAxis_L)
        # Right Thumbstick
        joystickAxis_R = -self.goodStick.getRightY()
        self.Climbgal_R.motorDirect(joystickAxis_R)

        # Making the encoder values corresponding to the drivers station or smth
        self.sd_table.putNumber("climber_R_motor_cmd", joystickAxis_R)
        self.sd_table.putNumber("climber_L_motor_cmd", joystickAxis_L)

        ### DISENGAGING AND ENGAGING RATCHET??? ###
        ratchet_engage_L = self.sd_table.getBoolean("ratchet_engage_L", False)
        ratchet_engage_R = self.sd_table.getBoolean("ratchet_engage_R", False)
        # Getting T/F from Network table
        if ratchet_engage_L == True:
            self.Climbgal_L.__engageRatchet__()
        else:
            self.Climbgal_L.__disengageRatchet__()

        if ratchet_engage_R == True:
            self.Climbgal_R.__engageRatchet__()
        else:
            self.Climbgal_R.__disengageRatchet__()

        if self.pxn_fightstick.getRawButtonPressed(7):
            self.Climbgal_L.findHomePosition()

        if self.pxn_fightstick.getRawButtonPressed(8):
            self.Climbgal_R.findHomePosition()


if __name__ == "__main__":
    wpilib.run(ReefscapeRobot)
