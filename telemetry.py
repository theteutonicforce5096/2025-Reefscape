from wpilib.shuffleboard import Shuffleboard
from wpilib import Field2d
from phoenix6 import swerve

class Telemetry:
    def __init__(self):
        """
        Construct a telemetry object.
        """

        # Field2d Widget
        self.field = Field2d()
        Shuffleboard.getTab("Drivers").add(f"Field", self.field).withSize(3, 3)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to ShuffleBoard.
        """
        # Telemeterize the pose to Field2d widget
        self.field.setRobotPose(state.pose)