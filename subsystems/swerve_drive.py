from commands2 import Subsystem
from commands2.cmd import sequence, waitSeconds
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from phoenix6 import swerve, SignalLogger

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

from subsystems.limelight import Limelight

from wpilib.shuffleboard import Shuffleboard
from wpilib import DriverStation, Field2d, SendableChooser

from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from math import radians, pi, cos, sin

class SwerveDrive(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants, modules,
                 robot_length, robot_distance_to_reef, robot_distance_to_coral):
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
        :param robot_length: Length of the robot in meters.
        :type robot_length: float
        :param robot_distance_to_reef: Distance in meters the robot needs to be away from reef for alignment to Reef
        :type robot_distance_to_reef: float
        :param robot_distance_to_coral: Distance in meters the robot needs to be away from reef for alignment to Reef
        :type robot_distance_to_coral: float
        :param robot_distance_to_coral: Distance in meters the robot needs to move from AprilTag to coral post on either side
        :type robot_distance_to_coral: float
        """

        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, modules)
        
        # Initialize Limelight and configure default values
        self.limelight = Limelight()
        self.limelight.set_limelight_network_table_entry_double("pipeline", 0)
        self.limelight.set_limelight_network_table_entry_double("imumode_set", 0)

        # Create Apply Robot Speeds Request for PathPlanner
        self.apply_robot_speeds_request = (
            swerve.requests.ApplyRobotSpeeds()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_desaturate_wheel_speeds(True)
        )

        AutoBuilder.configure(
            lambda: self.get_state_copy().pose,
            self.reset_pose,
            lambda: self.get_state_copy().speeds,
            lambda speeds, feedforwards: self.set_control(
                self.apply_robot_speeds_request
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                PIDConstants(10.0, 0.0, 0.0),
                PIDConstants(7.0, 0.0, 0.0)
            ),
            RobotConfig.fromGUISettings(),
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
            self
        )

        # Create Field2d Widget and add it to Shuffleboard
        self.field2d = Field2d()
        Shuffleboard.getTab("Pose Estimation").add(f"Estimated Pose", self.field2d).withSize(4, 2)

        # Create AprilTag Field and robot length variable for pose transformation
        self.april_tag_field = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)
        self.robot_length = robot_length
        self.robot_distance_to_reef = robot_distance_to_reef
        self.robot_distance_to_coral = robot_distance_to_coral

        # Swerve requests for SysId characterization
        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self.rotation_characterization = swerve.requests.SysIdSwerveRotation()

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

        self.sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = pi / 6,
                stepVoltage = 7.0,
                timeout = 5.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Rotation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.rotation_characterization.with_rotational_rate(output)),
                lambda log: None,
                self,
            ),
        )

        # Create widget for selecting SysId routine and set default value
        self.sys_id_routine_to_apply = self.sys_id_routine_translation
        self.sys_id_routines = SendableChooser()
        self.sys_id_routines.setDefaultOption("Translation Routine", self.sys_id_routine_translation)
        self.sys_id_routines.addOption("Steer Routine", self.sys_id_routine_steer)
        self.sys_id_routines.addOption("Rotation Routine", self.sys_id_routine_rotation)

        # Send widget to Shuffleboard 
        Shuffleboard.getTab("SysId").add(f"Routines", self.sys_id_routines).withSize(2, 1)

    def periodic(self):
        """
        Periodically called by CommandScheduler for updating drivetrain state.
        """

        # Set robot orientation in Limelight
        self.limelight.set_robot_orientation(self.get_state_copy().pose.rotation().degrees())

        # Get robot pose from limelight
        limelight_robot_pose, timestamp = self.limelight.get_robot_pose()

        # Add Limelight vision measurement to odometry
        self.add_limelight_vision_measurement(limelight_robot_pose, timestamp, 
                                              1.0, 45.0, (0.2, 0.2, 0.2))
        
        # Update odometry pose of robot in odometry Field 2d Widget
        self.field2d.setRobotPose(self.get_state_copy().pose)

    def apply_request(self, request):
        """
        Return a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def get_target_reef_pose(self, right_side):
        """
        Return the pose that the robot should target to the Reef that is the closest to its current position
        on either the left or right side of an AprilTag on the Reef.
        Returns None if Limelight on robot cannot detect an AprilTag on the Reef.

        :param right_side: Align robot to right side of AprilTag on Reef.
        :type right_side: bool
        """

        # Get the ID of the primary in-view AprilTag and get target side for alignment.
        tag_id = self.limelight.get_primary_apriltag_id(None)
        target_side = "Right" if right_side else "Left"

        # Return None if no AprilTag found otherwise get the pose of the AprilTag
        if tag_id == None:
            return None
        else:
            tag_pose = self.april_tag_field.getTagPose(tag_id).toPose2d()

        # Get vector components for vector that translates robot from AprilTag pose to front bumper touching Reef
        # assuming robot is orientated straight
        x_component_reef = self.robot_length / 2
        y_component_reef = 0.0

        # Get vector components for vector that translates robot to left or right side of AprilTag 
        # assuming robot is orientated straight
        x_component_coral = 0.0
        y_component_coral = self.robot_distance_to_coral

        # Get target pose
        if tag_id in [18, 10]:
            transform_front_bumper_to_reef = Transform2d(Pose2d(), Pose2d(-x_component_reef, y_component_reef, radians(180)))
            if target_side == "Left":
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, y_component_coral, 0))
            else:
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral -y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        elif tag_id in [21, 7]:    
            transform_front_bumper_to_reef = Transform2d(Pose2d(), Pose2d(x_component_reef, y_component_reef, radians(180)))
            if target_side == "Left":
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, -y_component_coral, 0))
            else:
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        else:
            # Get vector components for vector that translates robot from AprilTag pose to front bumper touching Reef
            # assuming robot is angled at 60 degrees
            x_component_reef = cos(radians(60)) * (self.robot_length / 2)
            y_component_reef = sin(radians(60)) * (self.robot_length / 2)

            # Get vector components for vector that translates robot to left or right side of AprilTag 
            # assuming robot is angled at 60 degrees
            x_component_coral = cos(radians(60)) * self.robot_distance_to_coral
            y_component_coral = sin(radians(60)) * self.robot_distance_to_coral

        if tag_id in [19, 9]:
            transform_front_bumper_to_reef = Transform2d(Pose2d(), Pose2d(-x_component_reef, y_component_reef, radians(180)))
            if target_side == "Left":
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, y_component_coral, 0))
            else:
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(-x_component_coral, -y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        elif tag_id in [20, 8]:
            transform_front_bumper_to_reef = Transform2d(Pose2d(), Pose2d(x_component_reef, y_component_reef, radians(180)))
            if target_side == "Left":
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, -y_component_coral, 0))
            else:
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(-x_component_coral, y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        elif tag_id in [17, 11]:
            transform_front_bumper_to_reef = Transform2d(Pose2d(), Pose2d(-x_component_reef, -y_component_reef, radians(180)))
            if target_side == "Left":
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(-x_component_coral, y_component_coral, 0))
            else:
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, -y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        elif tag_id in [22, 6]:
            transform_front_bumper_to_reef = Transform2d(Pose2d(), Pose2d(x_component_reef, -y_component_reef, radians(180)))
            if target_side == "Left":
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(-x_component_coral, -y_component_coral, 0))
            else:
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        else:
            return None

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

    def get_starting_position(self):
        """
        Get the initial pose that the robot should be at on the field when a match starts.
        """

        # Pull pose from pathplanner if set up
        # Since not yet set up or poses not set up based on driverstation
        # Use something random
        return Pose2d(1.112, 4.039, radians(0))
        
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
            translation_robot_pose_distance = odometry_robot_pose.translation().distance(limelight_robot_pose.translation())
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
