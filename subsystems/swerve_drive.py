import threading
import time

from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
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

from wpimath.geometry import Pose2d, Rotation2d
from math import pi, cos, sin

class SwerveDrive(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants, modules,
                 max_linear_speed, max_angular_rate, robot_length, robot_width, reef_spacing, coral_offset):
        """
        Constructor for initializing swerve drivetrain using the specified constants.

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
        :param robot_width: Width of the robot in meters.
        :type robot_width: float
        :param reef_spacing: Distance in meters the robot needs to be away from reef for alignment to Reef
        :type reef_spacing: float
        :param coral_offset: Distance in meters from AprilTag to coral post on either side
        :type coral_offset: float
        """

        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, modules)
        
        # Create Limelight instance and configure default values
        self.limelight = Limelight()
        self.limelight.set_limelight_network_table_entry_double("pipeline", 0)
        self.limelight.set_limelight_network_table_entry_double("imumode_set", 4)
        self.limelight.set_limelight_network_table_entry_double("imuassistalpha_set", 10**9) # Default: 0.001
        self.limelight.set_limelight_network_table_entry_double("ledMode", 3)
        
        # Create max speeds variables
        self.max_linear_speed = max_linear_speed
        self.max_angular_rate = max_angular_rate

        # Register telemetry function in swerve drive
        # self.register_telemetry(lambda state: self._odometry_thread_target(state))

        # Create vision thread and thread lock
        self.vision_thread = threading.Thread(target = self._vision_thread_target, daemon = True)
        self.thread_lock = threading.Lock()

        # Start vision thread
        # self.vision_thread.start()

        # Create Field2d Widget and add it to Shuffleboard
        self.field2d = Field2d()
        Shuffleboard.getTab("Autonomous").add(f"Estimated Pose", self.field2d).withSize(4, 2)

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
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed * 4, -self.max_linear_speed * 8)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed * 4, -self.max_linear_speed * 8)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate * 4, -self.max_angular_rate * 8)

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

        # Create robot variables for pose transformation for align function
        self.robot_length = robot_length
        self.reef_spacing = reef_spacing
        self.coral_offset = coral_offset

        # Create AprilTag Field and define Reef Tag IDs
        april_tag_field = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        reef_tag_ids = [17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11]

        # Create Reef AprilTag Pose List
        self.reef_tag_poses = []
        for reef_tag_id in reef_tag_ids:
            reef_tag_pose = april_tag_field.getTagPose(reef_tag_id).toPose2d()
            self.reef_tag_poses.append(reef_tag_pose)

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

        # Create widget for selecting Auto position and set default value
        self.auto_routines = SendableChooser()
        self.auto_routines.setDefaultOption("Blue 1", "Blue_1")
        self.auto_routines.addOption("Blue 2", "Blue_2")
        self.auto_routines.addOption("Blue 3", "Blue_3")
        self.auto_routines.addOption("Red 1", "Red_1")
        self.auto_routines.addOption("Red 2", "Red_2")
        self.auto_routines.addOption("Red 3", "Red_3")

        # Send widget to Shuffleboard 
        Shuffleboard.getTab("Autonomous").add(f"Auto Position", self.auto_routines).withSize(2, 1)

    def _vision_thread_target(self):
        """
        Add vision measurement to swerve drive every 20ms in vision thread.
        """

        # Get starting timestamp
        next_call = time.perf_counter()
        while True:
            # Call add vision measurement function
            self._add_vision_measurement()

            # Schedule the next call for 40ms later and calculate sleep time
            next_call += 0.04
            sleep_time = next_call - time.perf_counter()

            # Sleep if there is time until next call
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                pass

    def _odometry_thread_target(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Update pose of the robot in network tables periodically in odometry thread.

        :param state: State of the drivetrain
        :type state: swerve.SwerveDrivetrain.SwerveDriveState
        """
        # Get current pose of robot
        current_pose = state.pose

        # Set robot orientation in Limelight
        self.limelight.set_robot_orientation(current_pose.rotation().degrees())

        # Update pose of robot in Field 2d Widget
        self.field2d.setRobotPose(current_pose)

    def _add_vision_measurement(self):
        """
        Add vision measurement to swerve drive.
        """

        # Get a thread-safe copy of the current state of the drivetrain
        with self.thread_lock:
            current_state = self.get_state()

        # Get speed of robot in terms of percent of max speed from 0 to 1
        forward_speed = (abs(current_state.speeds.vx) / self.max_linear_speed)
        strafe_speed = (abs(current_state.speeds.vy) / self.max_linear_speed)
        rotation_speed = (abs(current_state.speeds.omega) / self.max_angular_rate)

        # Get highest speed
        highest_speed = max(forward_speed, strafe_speed, rotation_speed)

        # Clamp highest speed between 0 and 1
        highest_speed = min(max(highest_speed, 0.0), 1.0)
        
        # Get Limelight vision measurement
        limelight_robot_pose, timestamp = self.limelight.get_robot_pose()

        # Add Limelight vision measurement to odometry if pose or timestamp is not None 
        # and highest speed is not greater than 0.5
        if limelight_robot_pose == None or timestamp == None or highest_speed > 0.50:
            std_devs = None
        else:
            if 0.25 < highest_speed <= 0.50:
                highest_speed -= 0.25
                std_devs = (
                    5.0 + 5.0 * ((highest_speed / 0.25) ** 2), 
                    5.0 + 5.0 * ((highest_speed / 0.25) ** 2), 
                    50.0 + 50.0 * ((highest_speed / 0.25) ** 2)
                )
            elif 0.10 < highest_speed <= 0.25:
                highest_speed -= 0.10
                std_devs = (
                    2.0 + 3.0 * ((highest_speed / 0.15) ** 3), 
                    2.0 + 3.0 * ((highest_speed / 0.15) ** 3), 
                    20.0 + 30.0 * ((highest_speed / 0.15) ** 3)
                )
            elif highest_speed <= 0.10:
                std_devs = (
                    0.5 + 0.5 * (highest_speed / 0.10), 
                    0.5 + 0.5 * (highest_speed / 0.10), 
                    5.0 + 5.0 * (highest_speed / 0.10)
                )
            else: 
                std_devs = None

        if std_devs != None:
            with self.thread_lock:
                self.add_vision_measurement(
                    limelight_robot_pose,
                    timestamp,
                    std_devs
                )
    
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
    
    def get_operator_drive_command(self, left_trigger_pressed: bool, right_trigger_pressed: bool,
                                   forward_speed: float, strafe_speed: float, rotation_speed: float):
        """
        Get the desired drive command of the operator.

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
            operator_drive_request = (
                self.slow_mode_field_centric_request.with_velocity_x(
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
            )
        else:
            operator_drive_request = (
                self.default_mode_field_centric_request.with_velocity_x(
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
            )
        
        return self.runOnce(lambda: self.set_control(operator_drive_request))
    
    def _get_closest_reef_tag_pose(self, pose: Pose2d):
        """
        Return the pose of the AprilTag on the Reef that is the closest to the given pose.

        :param pose: Pose that is used for finding the pose of the nearest AprilTag on the Reef
        :type pose: Pose2d
        """
        return pose.nearest(self.reef_tag_poses)

    def _get_reef_alignment_pose(self, right_side: bool):
        """
        Return the pose that the robot should target to the Reef that is the closest to its current position
        on either the left or right side of an AprilTag on the Reef.

        :param right_side: Align robot to right side of AprilTag on Reef. 
        :type right_side: bool
        """
        # Get current pose and pose of closest AprilTag on Reef
        current_pose = self.get_state().pose
        closest_reef_tag_pose = self._get_closest_reef_tag_pose(current_pose)

        # Get target side for alignment
        target_side = "Right" if right_side else "Left"

        # Get the components of the pose
        x = closest_reef_tag_pose.X()
        y = closest_reef_tag_pose.Y()
        z = closest_reef_tag_pose.rotation().radians()

        # Translate AprilTag pose to center of robot with front bumper touching the middle of the Reef
        translated_x = x + ((self.robot_length / 2) * cos(z))
        translated_y = y + ((self.robot_length / 2) * sin(z))
        translated_z = z - pi

        # Calculate angle for coral offset
        coral_angle = z - (pi / 2)

        # Translate robot pose to left or right side of center of Reef
        if target_side == "Left":
            translated_x += self.coral_offset * cos(coral_angle) 
            translated_y += self.coral_offset * sin(coral_angle)
        elif target_side == "Right":
            translated_x -= self.coral_offset * cos(coral_angle)
            translated_y -= self.coral_offset * sin(coral_angle)

        # Translate robot pose away from Reef according to reef spacing variable
        translated_x -= self.reef_spacing * cos(translated_z)
        translated_y -= self.reef_spacing * sin(translated_z)
 
        return Pose2d(translated_x, translated_y, translated_z)

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
        :param controller: Controller that has a Y button that can stop alignment command
        :type controller: CommandXboxController
        :param right_side: Align robot to right side of AprilTag on Reef.
        :type right_side: bool
        """
        
        target_pose = self._get_reef_alignment_pose(right_side)
        return AutoBuilder.pathfindToPose(
            target_pose,
            path_constraints,
            goal_end_velocity
        ).until(
            lambda: controller.getHID().getYButtonPressed()
        )

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

        starting_position = self.auto_routines.getSelected()

        starting_poses = {
            "Blue_1": Pose2d(7.175, 6.162, Rotation2d.fromDegrees(0.000)),
            "Blue_2": Pose2d(7.175, 4.054, Rotation2d.fromDegrees(0.000)),
            "Blue_3": Pose2d(7.175, 1.889, Rotation2d.fromDegrees(0.000)),
            "Red_1": Pose2d(10.375, 1.894, Rotation2d.fromDegrees(180.000)),
            "Red_2": Pose2d(10.375, 4.047, Rotation2d.fromDegrees(180.000)),
            "Red_3": Pose2d(10.375, 6.162, Rotation2d.fromDegrees(180.000)),
        }

        return starting_poses[starting_position]

    def set_sys_id_routine(self):
        """
        Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        """
        self.sys_id_routine_to_apply = self.sys_id_routines.getSelected()

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Quasistatic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Dynamic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.dynamic(direction)
