from wpimath.geometry import Translation2d, Rotation2d
from wpimath.controller import PIDController
from wpilib.shuffleboard import Shuffleboard
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from swerve_module import SwerveModule
import navx 

class SwerveDrive():
    """Class for controlling Swerve Drive on robot."""

    def __init__(self):
        """
        Constructor for Swerve Drive.
        """        
        # Locations of Swerve Modules.
        front_left_location = Translation2d(0.250825, 0.250825)
        front_right_location = Translation2d(0.250825, -0.250825)
        back_left_location = Translation2d(-0.250825, 0.250825)
        back_right_location = Translation2d(-0.250825, -0.250825)

        # Create Kinematics object and initialize Swerve Modules
        self.kinematics = SwerveDrive4Kinematics(front_left_location, front_right_location, back_left_location, back_right_location)
        self.front_left_module = SwerveModule("FL", 23, 13, 33, "CANivore", "CANivore", "rio", -0.007080, False)
        self.front_right_module = SwerveModule("FR", 20, 10, 30, "CANivore", "CANivore", "rio", 0.012451, True)
        self.back_left_module = SwerveModule("BL", 22, 12, 32, "CANivore", "CANivore", "rio", -0.474609, False)
        self.back_right_module = SwerveModule("BR", 21, 11, 31, "CANivore", "CANivore", "rio", -0.011475, True)

        # Initialize Gyro
        self.gyro = navx.AHRS.create_spi()
        self.drivers_tab_gyro = Shuffleboard.getTab("Drivers").add(f"Current Robot Angle (From Gyro)", round(self.get_current_robot_angle(), 2)).withSize(2, 2).getEntry()

        # Max Drivetrain speed
        self.max_drivetrain_speed = 0.25
        self.drivers_tab_speed = Shuffleboard.getTab("Drivers").add(f"Max Swerve Drive Speed", self.max_drivetrain_speed).withSize(2, 2).getEntry()

        # Pid Controller for Aligning to Speaker
        self.align_to_speaker_controller = PIDController(1/20, 0, 0)
        self.align_to_speaker_controller.enableContinuousInput(-180, 180)
        self.align_to_speaker_controller.setTolerance(2)

        # Drivetrain state
        self.drivetrain_state = "Disabled"
        self.drivers_tab_state = Shuffleboard.getTab("Drivers").add(f"Swerve Drive State", self.drivetrain_state).withSize(2, 2).getEntry()

    def reset_drivetrain(self):
        """
        Reset Swerve Modules.
        """
        self.front_left_module.reset()
        self.front_right_module.reset()
        self.back_left_module.reset()
        self.back_right_module.reset()
        self.change_drivetrain_state("Enabled")

    def reset_gyro(self):
        """
        Reset Gyro.
        """
        self.gyro.reset()
        self.drivers_tab_gyro.setFloat(0)

    def change_max_drivetrain_speed(self, speed):
        """
        Change max drivetrain speed.
        """
        self.max_drivetrain_speed = speed
        self.drivers_tab_speed.setFloat(self.max_drivetrain_speed)

    def change_drivetrain_state(self, state):
        """
        Change the drivetrain's state.
        """
        self.drivetrain_state = state
        self.drivers_tab_state.setString(self.drivetrain_state)

    def get_drivetrain_state(self):
        """
        Get the drivetrain's state.
        """
        return self.drivetrain_state
    
    def set_align_to_speaker_controller(self, angle):
        """
        Sets the Align to Speaker PID Controller at a certain angle.
        """
        self.align_to_speaker_controller.setSetpoint(angle)

    def reached_align_to_speaker_goal(self):
        return self.align_to_speaker_controller.atSetpoint()
    
    def update_align_to_speaker_controller(self):
        rotation_speed = self.align_to_speaker_controller.calculate(self.get_current_robot_angle())
        if rotation_speed > 1:
            rotation_speed = 1
        elif rotation_speed < -1:
            rotation_speed = -1
            
        self.change_max_drivetrain_speed(0.25)
        self.move_robot(0, 0, rotation_speed)

    def get_current_robot_angle(self):
        """
        Get the current robot angle relative to the field using the gyro.
        """
        # Scale the gyro angle from -360 to 360 scale to -180 to 180 scale.
        current_robot_angle = (self.gyro.getAngle() * -1) % 360
        if current_robot_angle > 180: 
            current_robot_angle -= 360
        elif current_robot_angle < -180:
            current_robot_angle += 360

        return current_robot_angle

    def move_robot(self, forward_speed, strafe_speed, rotation_speed):
        """
        Move the robot by a forward speed, strafe speed, and rotation speed.
        Forward speed is positive toward the opponent's alliance station wall.
        Strafe speed is positive toward the left field boundary.
        Rotation speed is positive in the counterclockwise direction.
        """
        # Get desired Swerve Modules' speeds and angles.
        current_robot_angle = self.get_current_robot_angle()
        self.drivers_tab_gyro.setFloat(round(current_robot_angle, 2))
        robot_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward_speed * 5.21208, strafe_speed * 5.21208, rotation_speed * 14.6934998988, Rotation2d.fromDegrees(current_robot_angle))
        front_left_module_state, front_right_module_state, back_left_module_state, back_right_module_state = self.kinematics.desaturateWheelSpeeds(self.kinematics.toSwerveModuleStates(robot_speeds), 5.21208)
        
        # Optimize desired Swerve Modules' angles.
        front_left_module_state = front_left_module_state.optimize(front_left_module_state, self.front_left_module.current_angle)
        front_right_module_state = front_right_module_state.optimize(front_right_module_state, self.front_right_module.current_angle)
        back_left_module_state = back_left_module_state.optimize(back_left_module_state, self.back_left_module.current_angle)
        back_right_module_state = back_right_module_state.optimize(back_right_module_state, self.back_right_module.current_angle)

        # Set the Swerve Modules to the desired speeds and angles.
        self.front_left_module.set(self.max_drivetrain_speed * front_left_module_state.speed, front_left_module_state.angle)
        self.front_right_module.set(self.max_drivetrain_speed * front_right_module_state.speed, front_right_module_state.angle)
        self.back_left_module.set(self.max_drivetrain_speed * back_left_module_state.speed, back_left_module_state.angle)
        self.back_right_module.set(self.max_drivetrain_speed * back_right_module_state.speed, back_right_module_state.angle)

    def stop_robot(self):
        """
        Stop moving the robot and hold its current position.
        """
        # Stop all of the Swerve Modules. 
        self.front_left_module.stop()
        self.front_right_module.stop()
        self.back_left_module.stop()
        self.back_right_module.stop()