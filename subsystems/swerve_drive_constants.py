from phoenix6 import CANBus, configs, hardware, signals, swerve, units
from subsystems.swerve_drive import SwerveDrive
from wpimath.units import inchesToMeters

class SwerveDriveConstants:
    """
    Constants for Swerve Drive Generated by the Tuner X Swerve Project Generator
    https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    """

    # The steer motor uses any SwerveModule.SteerRequestType control request with the
    # output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    # Position Control
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_s(0.10)
        .with_k_p(100)
        .with_k_i(0)
        .with_k_d(0)
        .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
    )

    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    _drive_gains = (
        configs.Slot0Configs() 
        .with_k_s(0)
        .with_k_a(0)
        .with_k_v(0.124)
        .with_k_p(0.1)
        .with_k_i(0)
        .with_k_d(0)
    )

    # The stator current at which the wheels start to slip
    # Essentially used as stator current limit
    # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/mechanisms/swerve/swerve-builder-api.html
    # https://v6.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html#preventing-brownouts
    _slip_current: units.ampere = 120.0

    # Every 1 rotation of the azimuth results in _couple_ratio drive motor turns
    # https://www.chiefdelphi.com/t/kcoupleratio-in-ctre-swerve/483380
    # If you’ve already generated your TunerConstants and the generated coupling ratio is non-zero,
    # there is nothing more you need to do. 
    _couple_ratio = 5.4

    # Theoretical free speed (m/s) at 12 V applied output
    # https://wcproducts.com/collections/gearboxes/products/swerve-x2i
    speed_at_12_volts: units.meters_per_second = 5.63

    _drive_gear_ratio = 5.67
    _steer_gear_ratio = 12.1
    _wheel_radius: units.meter = inchesToMeters(2)

    _invert_left_side = False
    _invert_right_side = True

    _pigeon_id = 30

    # The closed-loop output type to use for the steer motors;
    # This affects the PID/FF gains for the steer motors
    # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
    _steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    # The closed-loop output type to use for the drive motors;
    # This affects the PID/FF gains for the drive motors
    _drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    # The type of motor used for the drive motor
    _drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED

    # The type of motor used for the drive motor
    _steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    # The remote sensor feedback type to use for the steer motors
    _steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

    # Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    # Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
    # https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/swerve/swerve_module_constants/index.html#phoenix6.swerve.swerve_module_constants.SwerveModuleConstants.drive_motor_initial_configs
    _drive_initial_configs = configs.TalonFXConfiguration()

    # Swerve azimuth does not require much torque output, so we can set a relatively low
    # stator current limit to help avoid brownouts without impacting performance.
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(60).with_stator_current_limit_enable(True)
    )

    _encoder_initial_configs = configs.CANcoderConfiguration()

    _pigeon_configs = configs.Pigeon2Configuration()

    # CAN bus that the devices are located on;
    # All swerve devices must share the same CAN bus
    canbus = CANBus("CANivore")

    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(_pigeon_id)
        .with_pigeon2_configs(_pigeon_configs)
    )

    _constants_creator: swerve.SwerveModuleConstantsFactory[configs.TalonFXConfiguration, configs.TalonFXConfiguration, configs.CANcoderConfiguration] = (
        swerve.SwerveModuleConstantsFactory()
        .with_drive_motor_gear_ratio(_drive_gear_ratio)
        .with_steer_motor_gear_ratio(_steer_gear_ratio)
        .with_coupling_gear_ratio(_couple_ratio)
        .with_wheel_radius(_wheel_radius)
        .with_steer_motor_gains(_steer_gains)
        .with_drive_motor_gains(_drive_gains)
        .with_steer_motor_closed_loop_output(_steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(_drive_closed_loop_output)
        .with_slip_current(_slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(_drive_motor_type)
        .with_steer_motor_type(_steer_motor_type)
        .with_feedback_source(_steer_feedback_type)
        .with_drive_motor_initial_configs(_drive_initial_configs)
        .with_steer_motor_initial_configs(_steer_initial_configs)
        .with_encoder_initial_configs(_encoder_initial_configs)
    )

    # Front Left
    _front_left_drive_motor_id = 10
    _front_left_steer_motor_id = 20
    _front_left_encoder_id = 0
    _front_left_encoder_offset: units.rotation = -0.468017578125
    _front_left_steer_motor_inverted = True
    _front_left_encoder_inverted = False

    _front_left_x_pos: units.meter = inchesToMeters(10.25)
    _front_left_y_pos: units.meter = inchesToMeters(10.25)

    # Front Right
    _front_right_drive_motor_id = 11
    _front_right_steer_motor_id = 21
    _front_right_encoder_id = 1
    _front_right_encoder_offset: units.rotation = -0.00146484375
    _front_right_steer_motor_inverted = True
    _front_right_encoder_inverted = False

    _front_right_x_pos: units.meter = inchesToMeters(10.25)
    _front_right_y_pos: units.meter = inchesToMeters(-10.25)

    # Back Left
    _back_left_drive_motor_id = 12
    _back_left_steer_motor_id = 22
    _back_left_encoder_id = 2
    _back_left_encoder_offset: units.rotation = -0.089111328125
    _back_left_steer_motor_inverted = True
    _back_left_encoder_inverted = False

    _back_left_x_pos: units.meter = inchesToMeters(-10.25)
    _back_left_y_pos: units.meter = inchesToMeters(10.25)

    # Back Right
    _back_right_drive_motor_id = 13
    _back_right_steer_motor_id = 23
    _back_right_encoder_id = 3
    _back_right_encoder_offset: units.rotation = 0.33642578125
    _back_right_steer_motor_inverted = True
    _back_right_encoder_inverted = False

    _back_right_x_pos: units.meter = inchesToMeters(-10.25)
    _back_right_y_pos: units.meter = inchesToMeters(-10.25)

    front_left = _constants_creator.create_module_constants(
        _front_left_steer_motor_id,
        _front_left_drive_motor_id,
        _front_left_encoder_id,
        _front_left_encoder_offset,
        _front_left_x_pos,
        _front_left_y_pos,
        _invert_left_side,
        _front_left_steer_motor_inverted,
        _front_left_encoder_inverted,
    )

    front_right = _constants_creator.create_module_constants(
        _front_right_steer_motor_id,
        _front_right_drive_motor_id,
        _front_right_encoder_id,
        _front_right_encoder_offset,
        _front_right_x_pos,
        _front_right_y_pos,
        _invert_right_side,
        _front_right_steer_motor_inverted,
        _front_right_encoder_inverted,
    )

    back_left = _constants_creator.create_module_constants(
        _back_left_steer_motor_id,
        _back_left_drive_motor_id,
        _back_left_encoder_id,
        _back_left_encoder_offset,
        _back_left_x_pos,
        _back_left_y_pos,
        _invert_left_side,
        _back_left_steer_motor_inverted,
        _back_left_encoder_inverted,
    )

    back_right = _constants_creator.create_module_constants(
        _back_right_steer_motor_id,
        _back_right_drive_motor_id,
        _back_right_encoder_id,
        _back_right_encoder_offset,
        _back_right_x_pos,
        _back_right_y_pos,
        _invert_right_side,
        _back_right_steer_motor_inverted,
        _back_right_encoder_inverted,
    )

    @classmethod
    def create_drivetrain(clazz) -> SwerveDrive:
        """
        Creates a SwerveDrive instance.
        """

        return SwerveDrive(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            clazz.drivetrain_constants,
            [
                clazz.front_left,
                clazz.front_right,
                clazz.back_left,
                clazz.back_right,
            ],
        )
