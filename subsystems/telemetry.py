from subsystems.limelight import Limelight

from phoenix6 import swerve

class Telemetry:
    def __init__(self):
        """
        Constructor for swerve drive telemetry.
        """

        # Create Limelight instance
        self.limelight = Limelight()

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to Shuffleboard.
        """
        # Set robot orientation in Limelight
        self.limelight.set_robot_orientation(state.pose.rotation().degrees())