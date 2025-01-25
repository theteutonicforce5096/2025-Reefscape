from ntcore import NetworkTableInstance
from wpilib import Field2d
from phoenix6 import swerve

class Telemetry:
    def __init__(self):
        """
        Construct a telemetry object.
        """

        self._instance = NetworkTableInstance.getDefault()

        # Robot pose
        self._table = self._instance.getTable("Pose")
        self._field_pub = self._table.getDoubleArrayTopic("robotPose").publish()
        self._field_type_pub = self._table.getStringTopic(".type").publish()

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to ShuffleBoard.
        """
        # Telemeterize the pose to Field2d 
        pose_array = [state.pose.x, state.pose.y, state.pose.rotation().degrees()]
        self._field_type_pub.set("Field2d")
        self._field_pub.set(pose_array)