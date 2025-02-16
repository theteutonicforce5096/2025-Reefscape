from ntcore import NetworkTableInstance

from phoenix6 import utils

from wpimath.geometry import Pose2d, Rotation2d

class Limelight:
    """
    Class for controlling Limelight.
    """

    def __init__(self):
        """
        Constructor for controlling Limelight.
        """
        
        # Get Limelight's network table
        self.limelight_network_table = NetworkTableInstance.getDefault().getTable("limelight")

    def set_limelight_network_table_entry_double(self, entry_name, value):
        """
        Set the value of a network table entry from Limelight.

        :parma entry_name: Name of the network table entry
        :type entry_name: str
        :param value: Value to set to the network table entry
        :type value: float
        """

        self.limelight_network_table.getEntry(entry_name).setDouble(value)

    def get_primary_apriltag_id(self, default_value):
        """
        Return the ID of the primary in-view AprilTag.

        :param default_value: Value to return if no value is found.
        :type default_value: int
        """

        return self.limelight_network_table.getEntry("tid").getInteger(default_value)

    def set_robot_orientation(self, yaw):
        """
        Set the orientation of the robot in Limelight.

        :param yaw: Current yaw of the robot in degrees
        :type yaw: float
        """
        self.limelight_network_table.getEntry("robot_orientation_set").setDoubleArray([yaw, 0, 0, 0, 0, 0])
        NetworkTableInstance.getDefault().flush()

    def get_robot_pose(self):
        """
        Get the current pose of the robot from Limelight.
        """

        # Get timestamped robot pose array
        robot_pose_timestamped_array = self.limelight_network_table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([-100]).getAtomic()
        robot_pose_array = robot_pose_timestamped_array.value
        fpga_timestamp_microseconds = robot_pose_timestamped_array.time

        if robot_pose_array[0] == -100 or fpga_timestamp_microseconds == 0:
            robot_pose = None
            timestamp = None
        else:
            # Get robot pose
            robot_pose = Pose2d(robot_pose_array[0], robot_pose_array[1], Rotation2d.fromDegrees(robot_pose_array[5]))

            if robot_pose == Pose2d(0, 0, 0):
                robot_pose = None
                timestamp = None
            else:
                # Get latency
                latency = robot_pose_array[6]

                # Convert server timestamp from microseconds to seconds and adjust for latency in ms.
                # Then, convert to timebase that phoenix6 uses.
                timestamp = utils.fpga_to_current_time((fpga_timestamp_microseconds / 1000000.0) - (latency / 1000.0))            

        # Return robot pose and timestamp
        return robot_pose, timestamp
