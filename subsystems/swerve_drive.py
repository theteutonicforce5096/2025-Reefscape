from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from commands2.cmd import print_
from commands2.button import CommandXboxController

from wpilib.sysid import SysIdRoutineLog

from phoenix6 import swerve, SignalLogger

from wpimath.filter import SlewRateLimiter

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pathplannerlib.auto import AutoBuilder, RobotConfig, PathConstraints

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
                 max_linear_speed, max_angular_rate, robot_length, robot_distance_to_reef, robot_distance_to_coral):
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
        :param max_linear_speed: Max linear speed of drivetrain in meters per second. 
        :type max_linear_speed: float
        :param max_angular_rate: Max angular velocity of drivetrain in radians per second. 
        :type max_angular_rate: float
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
        
        # Create max speeds variables
        self.max_linear_speed = max_linear_speed
        self.max_angular_rate = max_angular_rate

        # Create request for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.default_mode_field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.01)
            .with_rotational_deadband(self.max_angular_rate * 0.01)
            .with_desaturate_wheel_speeds(True)
        )

        self.slow_mode_field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.005)
            .with_rotational_deadband(self.max_angular_rate * 0.005)
            .with_desaturate_wheel_speeds(True)
        )

        # Create slew rate limiters for limiting robot acceleration
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed * 4, -self.max_linear_speed * 4)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed * 4, -self.max_linear_speed * 4)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate * 4, -self.max_angular_rate * 4)

        # Create Apply Robot Speeds Request for PathPlanner
        self.apply_robot_speeds_request = (
            swerve.requests.ApplyRobotSpeeds()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_desaturate_wheel_speeds(True)
        )

        # Configure PathPlanner
        AutoBuilder.configure(
            lambda: self.get_state().pose,
            self.reset_pose,
            lambda: self.get_state().speeds,
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
        self.april_tag_field = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
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
                stepVoltage = 4.0,
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
                stepVoltage = 4.0,
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

        # Get current state of the drivetrain
        current_state = self.get_state()

        # Set robot orientation in Limelight
        self.limelight.set_robot_orientation(current_state.pose.rotation().degrees())

        # Get speed of robot in terms of percent of max speed
        forward_speed = (abs(current_state.speeds.vx) / self.max_linear_speed) * 100
        strafe_speed = (abs(current_state.speeds.vy) / self.max_linear_speed) * 100
        rotation_speed = (abs(current_state.speeds.omega) / self.max_angular_rate) * 100

        # Get highest speed in terms of percent of max speed
        highest_speed = max(forward_speed, strafe_speed, rotation_speed)
        
        limelight_robot_pose, timestamp = self.limelight.get_robot_pose()

        # Add Limelight vision measurement to odometry if pose or timestamp is not None
        if limelight_robot_pose == None or timestamp == None:
            pass
        else:
            if highest_speed > 50:
                std_devs = (100.0, 100.0, 1000.0)
            elif 25 < highest_speed <= 50:
                std_devs = (10.0, 10.0, 100.0)
            elif 12.5 < highest_speed <= 25:
                std_devs = (1.0, 1.0, 10.0)
            elif highest_speed <= 12.5:
                std_devs = (0.5, 0.5, 5.0)

            self.add_vision_measurement(
                limelight_robot_pose,
                timestamp,
                std_devs
            )
            
        # Update odometry pose of robot in Field 2d Widget
        self.field2d.setRobotPose(self.get_state().pose)

    def apply_request(self, request):
        """
        Return a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))
    
    def reset_slew_rate_limiters(self):
        """
        Reset the slew rate limiters to the current speeds of the drivetrain.
        """

        # Get current state of the drivetrain
        current_state = self.get_state()

        # Reset slew rate limiters to current speed of the drivetrain
        self.straight_speed_limiter.reset(current_state.speeds.vx)
        self.strafe_speed_limiter.reset(current_state.speeds.vy)
        self.rotation_speed_limiter.reset(current_state.speeds.omega)
    
    def get_operator_drive_request(self, left_trigger_pressed: bool, right_trigger_pressed: bool,
                                   forward_speed: float, strafe_speed: float, rotation_speed: float):
        """
        Return the desired drive request of the operator.

        :param left_trigger_pressed: Whether the left trigger of the operator's controller is pressed or not.
        :type left_trigger_pressed: bool
        :param right_trigger_pressed: Whether the right trigger of the operator's controller is pressed or not.
        :type right_trigger_pressed: bool
        :param forward_speed: Desired forward speed of the operator in terms of percent of max linear speed where forward is positive. 
        :type forward_speed: float
        :param strafe_speed: Desired strafe speed of the operator in terms of percent of max linear speed where right is positive. 
        :type strafe_speed: float
        :param rotation_speed: Desired rotation speed of the operator in terms of percent of max angular speed where clockwise is positive. 
        :type rotation_speed: float
        """

        if left_trigger_pressed and right_trigger_pressed:
            return self.slow_mode_field_centric_request.with_velocity_x(
                self.straight_speed_limiter.calculate(
                    -(forward_speed * abs(forward_speed * 0.5)) * self.max_linear_speed
                )
            ).with_velocity_y(
                self.strafe_speed_limiter.calculate(
                    -(strafe_speed * abs(strafe_speed * 0.5)) * self.max_linear_speed
                )
            ).with_rotational_rate(
                self.rotation_speed_limiter.calculate(
                    -(rotation_speed * abs(rotation_speed * 0.5)) * self.max_angular_rate
                )
            )
        else:
            return self.default_mode_field_centric_request.with_velocity_x(
                self.straight_speed_limiter.calculate(
                    -(forward_speed * abs(forward_speed)) * self.max_linear_speed
                )
            ).with_velocity_y(
                self.strafe_speed_limiter.calculate(
                    -(strafe_speed * abs(strafe_speed)) * self.max_linear_speed
                )
            ).with_rotational_rate(
                self.rotation_speed_limiter.calculate(
                    -(rotation_speed * abs(rotation_speed)) * self.max_angular_rate
                )
            )

    def calculate_reef_alignment_pose(self, right_side: bool):
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
        if tag_id == None or tag_id == -1:
            return None
        else:
            tag_pose = self.april_tag_field.getTagPose(int(tag_id)).toPose2d()

        # Get vector components for vector that translates robot from AprilTag pose to front bumper touching Reef
        # assuming robot is orientated straight
        x_component_reef = (self.robot_length / 2) + self.robot_distance_to_reef
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
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, -y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        elif tag_id in [21, 7]:    
            transform_front_bumper_to_reef = Transform2d(Pose2d(), Pose2d(x_component_reef, y_component_reef, radians(180)))
            if target_side == "Left":
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, y_component_coral, 0))
            else:
                transform_robot_left_or_right = Transform2d(Pose2d(), Pose2d(x_component_coral, -y_component_coral, 0))

            return tag_pose.transformBy(transform_front_bumper_to_reef).transformBy(transform_robot_left_or_right)
        else:
            # Get vector components for vector that translates robot from AprilTag pose to front bumper touching Reef
            # assuming robot is angled at 60 degrees
            x_component_reef = cos(radians(60)) * ((self.robot_length / 2) + self.robot_distance_to_reef)
            y_component_reef = sin(radians(60)) * ((self.robot_length / 2) + self.robot_distance_to_reef)

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
        
    def get_reef_alignment_command(self, path_constraints: PathConstraints, 
                                   goal_end_velocity: float, controller: CommandXboxController, 
                                   right_side: bool):
        """
        Return the command that moves the robot to the side of the Reef 
        that is the closest to its current position
        on either the left or right side of an AprilTag on the Reef.

        :param path_constraints: Path constraints that the robot should follow while moving
        :type path_constraints: PathConstraints
        :param goal_end_velocity: Velocity the robot should have when reaching the target 
        :type goal_end_velocity: bool
        :param controller: Controller Y button that should stop Alignment Command
        :type controller: CommandXboxController
        :param right_side: Align robot to right side of AprilTag on Reef.
        :type right_side: bool
        """
        
        target_pose = self.calculate_reef_alignment_pose(right_side)
        if target_pose != None:
            return AutoBuilder.pathfindToPose(
                target_pose,
                path_constraints,
                goal_end_velocity
            ).until(
                lambda: controller.getHID().getYButtonPressed()
            )
        else:
            return print_("Reef Tag Not Found.")

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

        # Pull pose from pathplanner if auto paths set up
        # Pulling pose based on driverstation for now
        alliance = DriverStation.getAlliance()
        driver_station_number =  DriverStation.getLocation()
        
        if alliance == DriverStation.Alliance.kBlue:
            alliance = "Blue"
        else:
            alliance = "Red"
            
        if driver_station_number == 0 or driver_station_number == None:
            driver_station_number = 2

        starting_poses = {
            "Blue_1": Pose2d(7.175, 6.162, Rotation2d.fromDegrees(180.000)),
            "Blue_2": Pose2d(7.175, 4.054, Rotation2d.fromDegrees(180.000)),
            "Blue_3": Pose2d(7.175, 1.889, Rotation2d.fromDegrees(180.000)),
            "Red_1": Pose2d(10.375, 1.894, Rotation2d.fromDegrees(0.000)),
            "Red_2": Pose2d(10.375, 4.047, Rotation2d.fromDegrees(0.000)),
            "Red_3": Pose2d(10.375, 6.162, Rotation2d.fromDegrees(0.000)),
        }

        return starting_poses[f"{alliance}_{driver_station_number}"]

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
