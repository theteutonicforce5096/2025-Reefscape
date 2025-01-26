from phoenix6 import swerve
from wpilib import Field2d
from wpilib.shuffleboard import Shuffleboard

class Telemetry:
    def __init__(self):
        """
        Construct a telemetry object of the robot.
        https://www.chiefdelphi.com/t/fatal-python-error-segmentation-fault-when-using-register-telemetry-in-phoenix-6-swerve-drive-api/483721/4
        """
        self.field = Field2d()
        Shuffleboard.getTab("Drivers").add(f"Field", self.field).withSize(3, 3)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to Shuffleboard.
        """
        self.field.setRobotPose(state.pose)

