import limelight
import limelighthelpers

class Limelight:
    """
    Class for controlling Limelight.
    """

    def __init__(self):
        """
        Constructor for initializing Limelight.
        """

        self.limelight = limelight.Limelight("10.50.96.11")

        self.limelight.pipeline_switch(0)
        self.limelight.enable_websocket()

    def get_robot_pose(self):
        self.limelight.update_robot_orientation(yaw)