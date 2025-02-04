from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from phoenix6 import swerve, SignalLogger

from subsystems.limelight import Limelight

from wpilib.shuffleboard import Shuffleboard
from wpilib import DriverStation, Field2d, SendableChooser

from wpimath.geometry import Pose2d, Rotation2d
from math import radians

class SwerveDrive(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants, modules):
        """
        Constructs for initializing swerve drivetrain using the specified constants.

        :param drive_motor_type: Type of the drive motor
        :type drive_motor_type: type
        :param steer_motor_type: Type of the steer motor
        :type steer_motor_type: type
        :param encoder_type: Type of the azimuth encoder
        :type encoder_type: type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants: swerve.SwerveDrivetrainConstants
        :param modules: Constants for each specific module
        :type modules: list[swerve.SwerveModuleConstants]
        """

        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, modules)
        
        # Initialize Limelight and configure default values
        self.limelight = Limelight()
        self.limelight.set_limelight_network_table_entry_double("pipeline", 0)
        self.limelight.set_limelight_network_table_entry_double("imumode_set", 2)

        # Create Field2d Widgets and add them to Shuffleboard
        self.odometry_pose_field2d = Field2d()
        self.limelight_pose_field2d = Field2d()
        Shuffleboard.getTab("Pose Estimation").add(f"Odometry Pose Estimation", self.odometry_pose_field2d).withSize(3, 3)
        Shuffleboard.getTab("Pose Estimation").add(f"Limelight Pose Estimation", self.limelight_pose_field2d).withSize(3, 3)

        # Swerve requests for SysId characterization
        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()

        # Create SysId routine for characterizing drive.
        self.sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 4.0,
                timeout = 5.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Translation_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.translation_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # Create SysId routine for characterizing steer.
        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 7.0,
                timeout = 5.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Steer_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.steer_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # Create widget for selecting SysId routine and set default value
        self.sys_id_routine_to_apply = self.sys_id_routine_translation
        self.sys_id_routines = SendableChooser()
        self.sys_id_routines.setDefaultOption("Translation Routine", self.sys_id_routine_translation)
        self.sys_id_routines.addOption("Steer Routine", self.sys_id_routine_steer)

        # Send widget to Shuffleboard 
        Shuffleboard.getTab("SysId").add(f"Routines", self.sys_id_routines).withSize(2, 2)

    def periodic(self):
        """
        Periodically called by CommandScheduler for updating drivetrain state.
        """

        # Get robot pose from limelight
        limelight_robot_pose, timestamp = self.limelight.get_robot_pose()

        # Add Limelight vision measurement to odometry
        self.add_limelight_vision_measurement(limelight_robot_pose, timestamp, 
                                              1.0, 90.0, (0.5, 0.5, radians(45))) # (0.25, 0.25, radians(20))
        
        # Update odometry pose of robot in odometry Field 2d Widget
        self.odometry_pose_field2d.setRobotPose(self.get_state_copy().pose)

    def apply_request(self, request):
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))
    
    def set_forward_perspective(self):
        """
        Set forward perspective of the robot for field oriented drive.
        """

        alliance_color = DriverStation.getAlliance()
        if alliance_color is not None:
            if alliance_color == DriverStation.Alliance.kBlue:
                # Blue alliance sees forward as 0 degrees (toward red alliance wall)
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            else:
                # Red alliance sees forward as 180 degrees (toward blue alliance wall)
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))  

    def get_initial_robot_pose_match(self):
        """
        Get the initial pose that the robot should be at on the field when a match starts.
        """

        # Pull pose from pathplanner if set up
        # Since not yet set up or poses not set up based on driverstation
        # Use something random
        return Pose2d(0, 0, Rotation2d.fromDegrees(0))
    
    def set_limelight_robot_orientation(self, yaw):
        """
        Set the orientation of the robot in Limelight for Pose Estimation.
        """
        self.limelight.set_limelight_network_table_entry_double("imumode_set", 1)
        self.limelight.set_robot_orientation(yaw)
        self.limelight.set_limelight_network_table_entry_double("imumode_set", 2)

    def add_limelight_vision_measurement(self, limelight_robot_pose: Pose2d, timestamp, 
                                         max_translation_difference, max_rotation_difference, std_devs):
        """
        Add limelight vision measurement to odometry if valid and within a certain degree
        of current measurement to avoid bad vision data.

        :param limelight_robot_pose: Pose estimation of the robot from Limelight.
        :type limelight_robot_pose: Pose2d
        :param timestamp: Timestamp of the pose estimation
        :type timestamp: second
        :param max_translation_difference: Max translation difference between current pose reported by odometry 
        and Limelight pose that is accepted.
        :type max_translation_difference: meter
        :param max_rotation_difference: Max rotation difference between current pose reported by odometry 
        and Limelight pose that is accepted.
        :type max_rotation_difference: degree
        :param std_devs: Standard deviations of the robot pose measurement 
        (x position in meters, y position in meters, and heading in radians).
        :type std_devs: tuple[float, float, float]
        """

        # Don't try to add vision measurement if limelight robot pose or timestamp is None
        if limelight_robot_pose == None or timestamp == None:
            pass
        else:
            odometry_robot_pose = self.get_state_copy().pose

            # Calculate translation and rotation distance between the two poses
            translation_robot_pose_distance = odometry_robot_pose.translation().distance(limelight_robot_pose)
            rotation_robot_pose_distance = abs(
                odometry_robot_pose.rotation().degrees() - limelight_robot_pose.rotation().degrees()
            )

            # Only add vision measurement if limelight robot pose is within 
            # 1 meter translation and 90 degrees rotation of odometry robot pose
            if translation_robot_pose_distance <= max_translation_difference and rotation_robot_pose_distance <= max_rotation_difference:
                self.add_vision_measurement(
                    limelight_robot_pose,
                    timestamp,
                    std_devs
                )

            # Update limelight pose of robot in limelight Field 2d Widget
            self.limelight_pose_field2d.setRobotPose(limelight_robot_pose)

    def set_sys_id_routine(self):
        """
        Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        """
        self.sys_id_routine_to_apply = self.sys_id_routines.getSelected()

    def sys_id_quasistatic(self, direction):
        """
        Runs the SysId Quasistatic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction):
        """
        Runs the SysId Dynamic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.dynamic(direction)
