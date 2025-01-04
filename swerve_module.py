import phoenix6
from wpilib.shuffleboard import Shuffleboard
from wpimath.geometry import Rotation2d

class SwerveModule():
    """Class for controlling swerve module on robot."""

    def __init__(self, module_position, steering_motor_id, driving_motor_id, cancoder_id, steering_motor_bus, driving_motor_bus, cancoder_bus, cancoder_0_position_value, inverted_module):
        """
        Constructor for Swerve Module.

        :param module_position: Location of the module. Possible options: FR, FL, BR, BL
        :type module_position: str
        :param steering_motor_id: ID of the steering motor
        :type steering_motor_id: int
        :param driving_motor_id: ID of the driving motor
        :type driving_motor_id: int
        :param cancoder_id: ID of the CANcoder
        :type cancoder_id: int
        :param steering_motor_bus: CANbus the steering motor is on
        :type steering_motor_bus: str
        :param driving_motor_bus: CANbus the driving motor is on
        :type driving_motor_bus: str
        :param cancoder_0_position_value: Value of the CANcoder when the module is at 0 degrees. Used as a default for Shuffleboard widget.
        :type cancoder_0_position_value: float
        :param inverted_module: Whether the module is inverted or not.
        :type inverted_module: boolean
        """        
        # Hardware Initialization
        self.steering_motor = phoenix6.hardware.TalonFX(steering_motor_id, steering_motor_bus)
        self.driving_motor = phoenix6.hardware.TalonFX(driving_motor_id, driving_motor_bus)
        self.cancoder = phoenix6.hardware.CANcoder(cancoder_id, cancoder_bus)
        self.module_position = module_position

        # Steering Motor and Driving Motor Config
        self._configure_driving_motor(inverted_module)
        self._configure_steering_motor()

        # Cancoder Config
        self.cancoder_0_position_value = cancoder_0_position_value
        self._get_cancoder_0_position_value()
        
        # Swerve Module Configs
        self._determine_steering_motor_offset()   
        self.current_angle = None

    def _configure_driving_motor(self, inverted_module):
        """
        Configure the driving motor.
    
        :param inverted_module: Whether the module is inverted or not.
        :type inverted_module: bool
        """
        talonfx_configs = phoenix6.configs.TalonFXConfiguration()
        if inverted_module:
            talonfx_configs.motor_output.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        
        # Supply limit
        talonfx_configs.current_limits.stator_current_limit_enable = True
        talonfx_configs.current_limits.stator_current_limit = 55

        # PID Configs
        talonfx_configs.slot0.k_s = 0.18
        talonfx_configs.slot0.k_v = 0.13757366
        talonfx_configs.slot0.k_a = 0.01267852
        talonfx_configs.slot0.k_p = 0.11
        talonfx_configs.slot0.k_i = 0 
        talonfx_configs.slot0.k_d = 0 

        # Motion Magic
        talonfx_configs.motion_magic.motion_magic_acceleration = 100
        talonfx_configs.motion_magic.motion_magic_jerk = 1500

        # Apply the configs to the driving motor
        self.driving_motor.configurator.apply(talonfx_configs) 

        # Create PID object
        self.driving_pid = phoenix6.controls.MotionMagicVelocityVoltage(velocity = 0, enable_foc = False)

    def _configure_steering_motor(self):
        """
        Configure the steering motor.
        """
        # Steering Motor Configs
        talonfx_configs = phoenix6.configs.TalonFXConfiguration()
        talonfx_configs.closed_loop_general.continuous_wrap = True
        talonfx_configs.feedback.sensor_to_mechanism_ratio = 150 / 7
        
        # Supply limit
        talonfx_configs.current_limits.stator_current_limit_enable = True
        talonfx_configs.current_limits.stator_current_limit = 25

        # PID Configs
        talonfx_configs.slot0.k_p = 100
        talonfx_configs.slot0.k_i = 0
        talonfx_configs.slot0.k_d = 0

        # Apply the configs to the steering motor
        self.steering_motor.configurator.apply(talonfx_configs) 

        # Create PID object
        self.steering_pid = phoenix6.controls.PositionVoltage(position = 0, enable_foc = False)

    def _get_cancoder_0_position_value(self):
        """
        Get the CANcoder value when the module is at 0 degrees from Shuffleboard.
        """
        self.cancoder_0_position_value = Shuffleboard.getTab("Swerve Drive").add(f"{self.module_position} 0 Position CANcoder Value", self.cancoder_0_position_value).withSize(2, 2).getEntry().getFloat(self.cancoder_0_position_value)  

    def _determine_steering_motor_offset(self):
        """
        Determine the steering motor offset to allow the swerve module to face forward.
        """
        # Calculate CANcoder offsets for 0 degree position
        cancoder_offset = self.cancoder_0_position_value - self.cancoder.get_absolute_position().value

        # Set steering motor offset
        self.steering_motor_offset = self.steering_motor.get_position().value - cancoder_offset

    def reset(self):
        """
        Reset the swerve module's main variables and set it to hold its position.
        """
        self._get_cancoder_0_position_value()
        self._determine_steering_motor_offset()
        self.current_angle = Rotation2d.fromDegrees(0)
        self.driving_motor.set_control(self.driving_pid.with_velocity(0))
        self.steering_motor.set_control(self.steering_pid.with_position(self.steering_motor_offset)) 

    def stop(self):
        """
        Stop the driving motor but continue to hold the last position received from the joystick.
        """
        self.driving_motor.set_control(self.driving_pid.with_velocity(0))

    def set(self, speed, angle):
        """
        Set the Swerve Module to a desired speed and angle.
    
        :param speed: Desired speed of the module in meters per second.
        :type speed: float
        :param angle: Desired angle of the module. Must already be optimized. Use wpimath.kinematics.SwerveModuleState.optimize to optimize the angle.
        :type angle: float
        """
        # Determine speed, position, and module direction
        desired_speed = speed
        desired_angle = ((angle.degrees() * -1) + 360) % 360
        desired_position = self.steering_motor_offset + (desired_angle / 360)
        self.current_angle = angle

        # Set the motors to the desired speed and angle
        self.driving_motor.set_control(self.driving_pid.with_velocity((desired_speed / 5.21208) * 100))
        self.steering_motor.set_control(self.steering_pid.with_position(desired_position)) 