from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from phoenix6 import SignalLogger, swerve

from wpilib import DriverStation
from wpilib.shuffleboard import Shuffleboard
from wpilib import SendableChooser

from math import pi
from wpimath.geometry import Rotation2d

from wpilib import Field2d

class SwerveDrive(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants, modules):
        """
        Constructs a swerve drivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

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
        
        # Create Field2d Widget in Shuffleboard
        self.field = Field2d()
        Shuffleboard.getTab("Drivers").add(f"Field", self.field).withSize(3, 3)

        # Swerve requests for SysId characterization
        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self.rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # SysId routine for characterizing drive.
        self.sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 4.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.translation_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing steer.
        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 7.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.steer_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine to test
        self.sys_id_routines = SendableChooser()
        self.sys_id_routines.setDefaultOption("Steer Routine", self.sys_id_routine_steer)
        self.sys_id_routines.addOption("Translation Routine", self.sys_id_routine_translation)

        Shuffleboard.getTab("SysId").add(f"Routines", self.sys_id_routines).withSize(2, 2)
        self.sys_id_routine_to_apply = self.sys_id_routines.getSelected()

    def periodic(self):
        self.update_pose(self.get_state())

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

    def update_pose(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to Shuffleboard.
        https://www.chiefdelphi.com/t/fatal-python-error-segmentation-fault-when-using-register-telemetry-in-phoenix-6-swerve-drive-api/483721/4
        """
        self.field.setRobotPose(state.pose)

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
