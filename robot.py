import wpilib
from subsystems.drivetrain import SwerveDrive
from subsystems.shooter import Shooter
from subsystems.arm import Arm
from subsystems.photoelectric_sensor import PhotoelectricSensor
from subsystems.vision import Vision

class TheRinger(wpilib.TimedRobot):
    def robotInit(self):
        # Set brownout voltage
        wpilib.RobotController.setBrownoutVoltage(6.3)

        # Initialize components
        self.drivetrain = SwerveDrive()
        self.shooter = Shooter(40, 41, 42, True, False, False)
        self.arm = Arm(50, 51, True, False, 0, 0.16469017911725448)
        self.photoelectric_sensor = PhotoelectricSensor(1)
        self.vision = Vision()

        # Initialize controllers
        self.drivetrain_controller = wpilib.XboxController(0)
        self.shooter_controller = wpilib.XboxController(1)

        # Initialize timers
        self.timer = wpilib.Timer()
        self.drivetrain_timer = wpilib.Timer()
        self.drop_timer = wpilib.Timer()
        self.prime_shooter_timer = wpilib.Timer()
        self.shoot_timer = wpilib.Timer()        
        self.ready_robot_timer = wpilib.Timer()
        self.auto_timer = wpilib.Timer()

        # Set default robot speeds
        self.forward_speed = 0
        self.strafe_speed = 0
        self.rotation_speed = 0
        
        # Location of robot
        if wpilib.DriverStation.getLocation() == 2:
            self.location = 'center'
        else:
            self.location = 'side'

        # State of autonomous
        self.autonomous_state = "None"

    def autonomousInit(self):
        # Reset timers
        self.timer.restart()
        self.drivetrain_timer.reset()
        self.drop_timer.reset()
        self.prime_shooter_timer.reset()
        self.shoot_timer.reset()
        self.ready_robot_timer.reset() 
        self.auto_timer.reset()

        # Reset robot speeds.
        self.forward_speed = 0
        self.strafe_speed = 0
        self.rotation_speed = 0

        # Location of robot
        if wpilib.DriverStation.getLocation() == 2:
            self.location = 'center'
        elif wpilib.DriverStation.getLocation() == 1:
            self.location = 'left'
        elif wpilib.DriverStation.getLocation() == 3:
            self.location = 'right'

        # State of autonomous
        self.autonomous_state = "Idle"

        # Reset Drivetrain
        self.drivetrain.reset_drivetrain()
        self.drivetrain.reset_gyro()
            
        # Reset shooter
        self.shooter.reset()

        # Reset Arm
        self.arm.reset()

        # Reset Vision
        self.vision.reset()
        pass

    def autonomousPeriodic(self):
        match self.autonomous_state:
            case "Idle":
                self.autonomous_state = "Fire Note"
            case "Fire Note":
                distance, yaw = self.vision.get_data_to_speaker()
                if distance != None and yaw != None:
                    arm_angle, flywheel_speed = self.shooter.predict_speaker_shooting_state(distance)
                    self.shooter.set_flywheel_motors(flywheel_speed)
                    self.prime_shooter_timer.restart()
                    
                    self.arm.set_speaker_shooting_position(arm_angle)
                    self.ready_robot_timer.restart()
                    self.autonomous_state = "Moving Arm"
                else:
                    self.shooter.set_flywheel_motors(0.8)
                    self.prime_shooter_timer.restart()
                    
        #             self.arm.set_speaker_shooting_position(-12.5)
        #             self.ready_robot_timer.restart()
        #             self.autonomous_state = "Moving Arm"
        #     case "Moving Arm":
        #         if self.ready_robot_timer.hasElapsed(3):
        #             self.ready_robot_timer.reset()
        #             self.autonomous_state = "Armed"
        #         elif self.arm.reached_goal():
        #             self.ready_robot_timer.reset()
        #             self.autonomous_state = "Armed"
        #     case "Armed":
        #         if self.prime_shooter_timer.hasElapsed(1.5):
        #             self.shooter.set_intake_motor(1)
        #             self.prime_shooter_timer.reset()
        #             self.shoot_timer.restart()
        #             self.autonomous_state = "Fire"
        #     case "Fire":
        #         if self.shoot_timer.hasElapsed(0.5):
        #             self.shoot_timer.reset()
        #             self.autonomous_state = "Reset Shooter"
        #     case "Reset Shooter":
        #         self.arm.set_carry_position()
        #         self.shooter.reset()  
        #         self.autonomous_state = "Stop Robot" 
        #         # if self.location == 'center':  
        #         #     self.autonomous_state = "Move Robot"
        #         # elif self.location == 'left':
        #         #     self.autonomous_state = "Move Robot/Rotate Right"
        #         # elif self.location == 'right':
        #         #     self.autonomous_state = "Move Robot/Rotate Left"
        #     case "Move Robot":
        #         self.drivetrain.move_robot(-1, 0, 0)
        #         self.auto_timer.restart()
        #         self.autonomous_state = "Moving Robot"
        #     case "Move Robot/Rotate Right":
        #         self.drivetrain.move_robot(-1, 0, -0.5)
        #         self.auto_timer.restart()
        #         self.autonomous_state = "Moving Robot"
        #     case "Move Robot/Rotate Left":
        #         self.drivetrain.move_robot(-1, 0, 0.5)
        #         self.auto_timer.restart()
        #         self.autonomous_state = "Moving Robot"
        #     case "Moving Robot":
        #         if self.auto_timer.hasElapsed(2):
        #             self.drivetrain.stop_robot()
        #             self.auto_timer.reset()
        #             self.autonomous_state = "Stop Robot"
        #     case "Stop Robot":
        #         pass

        self.arm.update_pid_controller()  
        pass   

    def teleopInit(self):
        # Reset timers
        self.timer.restart()
        self.drivetrain_timer.reset()
        self.drop_timer.reset()
        self.prime_shooter_timer.reset()
        self.shoot_timer.reset()
        self.ready_robot_timer.reset()

        # Reset robot speeds.
        self.forward_speed = 0
        self.strafe_speed = 0
        self.rotation_speed = 0
        
        # Reset Drivetrain
        self.drivetrain.reset_drivetrain()
        self.drivetrain.reset_gyro()
        
        # Reset shooter
        self.shooter.reset()

        # Reset Arm
        self.arm.reset()

        # Reset Vision
        self.vision.reset()

    def teleopPeriodic(self):
        # Get speeds from drivetrain controller.
        forward_speed = self.drivetrain_controller.getLeftY()
        strafe_speed = self.drivetrain_controller.getLeftX()
        rotation_speed = self.drivetrain_controller.getRightX()
        
        # Check if gyro needs to be reset.
        if self.drivetrain_controller.getAButtonPressed() and self.drivetrain_controller.getLeftBumperPressed() and self.drivetrain_controller.getRightBumperPressed():
            self.drivetrain.stop_robot()
            self.drivetrain.reset_gyro()
            self.drivetrain_timer.restart()
            self.drivetrain.change_drivetrain_state("Resetting Gyro")
        
        # Check if max drivetrain speed needs to be changed.
        if self.drivetrain_controller.getLeftTriggerAxis() > 0.1 and self.drivetrain_controller.getRightTriggerAxis() > 0.1:
            self.drivetrain.change_max_drivetrain_speed(1.0)
        elif self.drivetrain_controller.getLeftTriggerAxis() > 0.1:
            self.drivetrain.change_max_drivetrain_speed(0.5)
        elif self.drivetrain_controller.getRightTriggerAxis() > 0.1:
            self.drivetrain.change_max_drivetrain_speed(0.75)
        else:
            self.drivetrain.change_max_drivetrain_speed(0.25)

        # Set deadzone on forward speed.
        if forward_speed > 0.1 or forward_speed < -0.1:
            self.forward_speed = forward_speed
        else:
            self.forward_speed = 0

        # Set deadzone on strafe speed.
        if strafe_speed > 0.1 or strafe_speed < -0.1:
            self.strafe_speed = strafe_speed
        else:
            self.strafe_speed = 0

        # Set deadzone on rotation speed.
        if rotation_speed > 0.1 or rotation_speed < -0.1:
            self.rotation_speed = rotation_speed
        else:
            self.rotation_speed = 0

        #Check for shooter state overrides
        if self.shooter_controller.getYButtonPressed():
            self.shooter.change_shooter_state("Reset")
        elif self.shooter_controller.getRightBumperPressed():
            self.shooter.change_shooter_state("Force Fire")
        elif self.shooter_controller.getLeftBumperPressed():
            self.shooter.change_shooter_state("Release Note")

        # Check for arm setpoint change
        pov = self.shooter_controller.getPOV()
        if pov == 0:
            self.arm.set(self.arm.get_arm_setpoint() + 0.5)
        elif pov == 180:
            self.arm.set(self.arm.get_arm_setpoint() - 0.5)

        # Evaluate shooter state
        match self.shooter.get_shooter_state():
            case "Idle":
                if self.shooter_controller.getAButtonPressed():
                    if self.photoelectric_sensor.detects_ring():
                        self.shooter.change_shooter_state("Loaded")
                    else:
                        self.arm.set_collecting_position()
                        self.shooter.change_next_shooter_state("Start Collecting")
                        self.shooter.change_shooter_state("Moving Arm")
            case "Start Collecting":
                self.shooter.set_intake_motor(1)
                self.shooter.change_next_shooter_state("None")
                self.shooter.change_shooter_state("Collecting")
            case "Collecting":
                if self.photoelectric_sensor.detects_ring():
                    self.shooter.set_intake_motor(0)
                    self.arm.set_carry_position()
                    self.shooter.change_shooter_state("Loaded")
                    self.shooter.set_flywheel_motors(0.5)
            case "Loaded":
                if self.shooter_controller.getXButtonPressed():
                    self.drivetrain.stop_robot()
                    self.drivetrain.change_drivetrain_state("Disabled")

                    self.arm.set_amp_shooting_position()
                    self.shooter.set_flywheel_motors(0.5)

                    self.prime_shooter_timer.restart()
                    self.ready_robot_timer.restart()

                    self.shooter.change_next_shooter_state("Armed")
                    self.shooter.change_shooter_state("Ready Robot For Amp")
                elif self.shooter_controller.getBButtonPressed():
                    self.drivetrain.stop_robot()
                    distance, yaw = self.vision.get_data_to_speaker()
                    if distance != None and yaw != None:
                        self.drivetrain.change_drivetrain_state("Disabled")
                        arm_angle, flywheel_speed = self.shooter.predict_speaker_shooting_state(distance)
                        #if yaw != 0:
                        #    self.drivetrain.set_align_to_speaker_controller(self.drivetrain.get_current_robot_angle() + yaw)
                        #else:
                        #    self.drivetrain.set_align_to_speaker_controller(self.drivetrain.get_current_robot_angle())

                        self.shooter.set_flywheel_motors(flywheel_speed)
                        self.prime_shooter_timer.restart()
                        
                        self.arm.set_speaker_shooting_position(arm_angle)
                        self.ready_robot_timer.restart()

                        self.shooter.change_next_shooter_state("Armed")
                        self.shooter.change_shooter_state("Ready Robot For Speaker")
            case "Armed":
                if self.prime_shooter_timer.hasElapsed(1):
                    self.shooter.set_intake_motor(1)
                    self.prime_shooter_timer.reset()
                    self.shoot_timer.restart()
                    self.shooter.change_next_shooter_state("None")
                    self.shooter.change_shooter_state("Fire")
            case "Fire":
                if self.shoot_timer.hasElapsed(1):
                    self.shoot_timer.reset()
                    self.shooter.change_shooter_state("Reset")
            case "Force Fire":
                self.shooter.set_flywheel_motors(0.85)
                self.prime_shooter_timer.restart()
                self.shooter.change_shooter_state("Armed")
            case "Release Note":
                self.arm.set_carry_position()
                self.shooter.change_next_shooter_state("Start Releasing Note")
                self.shooter.change_shooter_state("Moving Arm")
            case "Start Releasing Note":
                self.shooter.set_flywheel_motors(-1)
                self.shooter.set_intake_motor(-1)
                self.drop_timer.restart()
                self.shooter.change_next_shooter_state("None")
                self.shooter.change_shooter_state("Stop Releasing Note")
            case "Stop Releasing Note":
                if self.drop_timer.hasElapsed(0.75):
                    self.drop_timer.reset()
                    self.shooter.change_shooter_state("Reset")
            case "Reset":
                self.drivetrain.change_drivetrain_state("Enabled")
                self.arm.set_carry_position()
                self.shooter.reset()
            case "Moving Arm":
                if self.arm.reached_goal():
                    self.shooter.change_shooter_state(self.shooter.get_next_shooter_state())
            case "Ready Robot For Amp":
                if self.ready_robot_timer.hasElapsed(1.5):
                    self.ready_robot_timer.reset()
                    self.shooter.change_shooter_state(self.shooter.get_next_shooter_state())
                else:
                    if self.arm.reached_goal():
                        self.ready_robot_timer.reset()
                        self.shooter.change_shooter_state(self.shooter.get_next_shooter_state())
            case "Ready Robot For Speaker":
                if self.ready_robot_timer.hasElapsed(1.5):
                    self.drivetrain.stop_robot()
                    self.ready_robot_timer.reset()
                    self.shooter.change_shooter_state(self.shooter.get_next_shooter_state())
                elif self.arm.reached_goal():#self.drivetrain.reached_align_to_speaker_goal() and self.arm.reached_goal():
                    self.drivetrain.stop_robot()
                    self.ready_robot_timer.reset()
                    self.shooter.change_shooter_state(self.shooter.get_next_shooter_state())
                #else:
                #    if self.drivetrain.reached_align_to_speaker_goal():
                #        self.drivetrain.stop_robot()
                #    else:
                #        self.drivetrain.update_align_to_speaker_controller()

        # Update arm position
        self.arm.update_pid_controller() 

        # Evaluate drivetrain state.
        match self.drivetrain.get_drivetrain_state():
            case "Enabled":
                if self.forward_speed != 0 or self.strafe_speed != 0 or self.rotation_speed != 0:
                    self.drivetrain.move_robot(self.forward_speed, self.strafe_speed, self.rotation_speed) 
                else:
                    self.drivetrain.stop_robot()
            case "Resetting Gyro":
                if not self.drivetrain_timer.hasElapsed(0.5):
                    if self.forward_speed != 0 or self.strafe_speed != 0 or self.rotation_speed != 0:
                        self.drivetrain_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0.75)  
                    else:
                        self.drivetrain_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)  
                else:
                    self.drivetrain_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)  
                    self.drivetrain_timer.reset()
                    self.drivetrain.change_drivetrain_state("Enabled")
            case "Disabled":
                if self.forward_speed != 0 or self.strafe_speed != 0 or self.rotation_speed != 0:
                    self.drivetrain_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0.75)
                else:
                    self.drivetrain_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)

    def teleopExit(self):
        # Turn off drivetrain controller rumble if it is stil on.
        self.drivetrain_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)  

    def testInit(self):
        #self.arm.set(0)
        pass

    def testPeriodic(self):
        # pov = self.shooter_controller.getPOV()
        # if pov == 0:
        #    self.arm.set(self.arm.get_arm_setpoint() + 0.5)
        # elif pov == 180:
        #    self.arm.set(self.arm.get_arm_setpoint() - 0.5)

        # self.arm.update_pid_controller() 
        print(self.arm._get_encoder_value())

if __name__ == "__main__":
    wpilib.run(TheRinger)