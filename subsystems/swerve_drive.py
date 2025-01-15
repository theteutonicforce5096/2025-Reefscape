from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from math import pi
from phoenix6 import SignalLogger, swerve
from wpilib.sysid import SysIdRoutineLog

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

        :param drive_motor_type:     Type of the drive motor
        :type drive_motor_type:      type
        :param steer_motor_type:     Type of the steer motor
        :type steer_motor_type:      type
        :param encoder_type:         Type of the azimuth encoder
        :type encoder_type:          type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
    
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, modules)

        # Swerve requests for SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # SysId routine for characterizing drive.
        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 4.0
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self._translation_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing steer.
        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 7.0,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self._steer_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing rotation. This is used for FieldCentricFacingAngle HeadingController.
        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate = pi / 6,
                stepVoltage = 7.0,
            ),
            SysIdRoutine.Mechanism(
                # output is actually radians per second, but SysId only supports "volts"
                lambda output: self.set_control(self._rotation_characterization.with_rotational_rate(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine to test
        self._sys_id_routine_to_apply = self._sys_id_routine_translation

    def apply_request( self, request ):
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction):
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction):
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)
