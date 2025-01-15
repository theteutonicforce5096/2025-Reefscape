from ntcore import NetworkTableInstance
from phoenix6 import swerve
class Telemetry:
    def __init__(self):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        # What to publish over networktables for telemetry
        self._inst = NetworkTableInstance.getDefault()

        # Robot pose for field positioning
        self._table = self._inst.getTable("Drivers")
        self._field_pub = self._table.getDoubleArrayTopic("robotPose").publish()
        self._field_type_pub = self._table.getStringTopic(".type").publish()

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to ShuffleBoard.
        """
        # Telemeterize the pose to a Field2d
        pose_array = [state.pose.x, state.pose.y, state.pose.rotation().degrees()]

        self._field_type_pub.set("Field2d")
        self._field_pub.set(pose_array)