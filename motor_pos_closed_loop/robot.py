"""Test Script of Steer Motors"""
import wpilib
import math
import constants

from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds

from phoenix6 import hardware, signals, controls, configs, StatusCode

class MyRobot(wpilib.TimedRobot):
    """
    Test pos CL control
    """

    def robotInit(self):
        self.FR_steer_motor = hardware.TalonFX(12, "*")
        self.FR_drive_motor = hardware.TalonFX(10, "*")
        self.FL_steer_motor = hardware.TalonFX(1, "*")
        self.FL_drive_motor = hardware.TalonFX(0, "*")
        self.BR_steer_motor = hardware.TalonFX(7, "*")
        self.BR_drive_motor = hardware.TalonFX(6, "*")
        self.BL_steer_motor = hardware.TalonFX(4, "*")
        self.BL_drive_motor = hardware.TalonFX(3, "*")

        # self.encoder
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        # Start at velocity 0, use slot 1
        self.velocity_torque = controls.VelocityTorqueCurrentFOC(0).with_slot(1)

        # Start at position 0, use slot 0
        self.position_voltage = controls.PositionVoltage(0).with_slot(0)
        # Start at position 0, use slot 1
        self.position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(1)
        # Keep a brake request so we can disable the motor
        self.brake = controls.NeutralOut()

        XboxController = wpilib.XboxController
        self.joystick = XboxController(0)
        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_p = 2; # An error of 1 rotation results in 2.4 V output
        cfg.slot0.k_i = 0.01; # No output for integrated error
        cfg.slot0.k_d = 0.05; # A velocity of 1 rps results in 0.1 V output  # 0.1
        cfg.slot0.k_v = 0.12
        # Peak output of 8 V
        cfg.voltage.peak_forward_voltage = 8
        cfg.voltage.peak_reverse_voltage = -8

        # cfg.slot1.k_p = 60; # An error of 1 rotation results in 60 A output
        # cfg.slot1.k_i = 0; # No output for integrated error
        # cfg.slot1.k_d = 6; # A velocity of 1 rps results in 6 A output
        # # Peak output of 120 A
        # cfg.torque_current.peak_forward_torque_current = 120
        # cfg.torque_current.peak_reverse_torque_current = -120

        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.FR_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        self.FR_steer_motor.set_position(0)

        for _ in range(0, 5):
            status = self.FL_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        # Make sure we start at 0
        self.FL_steer_motor.set_position(0)

        
        for _ in range(0, 5):
            status = self.BR_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        self.BR_steer_motor.set_position(0)

        
        for _ in range(0, 5):
            status = self.BL_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        self.BL_steer_motor.set_position(0)
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.FR_drive_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        for _ in range(0, 5):
            status = self.FL_drive_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        for _ in range(0, 5):
            status = self.BR_drive_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
            
        for _ in range(0, 5):
            status = self.BL_drive_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.FR_drive_motor.set_control(controls.Follower(self.FR_steer_motor.device_id, False))
        self.FL_drive_motor.set_control(controls.Follower(self.FL_steer_motor.device_id, False))
        self.BR_drive_motor.set_control(controls.Follower(self.BR_steer_motor.device_id, False))
        self.BL_drive_motor.set_control(controls.Follower(self.BL_steer_motor.device_id, False))
    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # TODO: control self.steer motor to reach "angle" (angle->position closed loop)
        # Go for plus/minus 10 rotations
        joy_value = self.joystick.getLeftTriggerAxis()
        if abs(joy_value) < 0.1:
            joy_value = 0

        # Go for plus/minus 50 rotations per second
        desired_rotations_per_second = joy_value * 50


        
        desired_rotations = self.joystick.getLeftY() * 10
        if abs(desired_rotations) <= 0.1: # Joystick deadzone
            desired_rotations = 0

        if self.joystick.getLeftBumper():
            # Use position voltage
            self.FR_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
            self.FL_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
            self.BR_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
            self.BL_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
            self.FR_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))
            self.FL_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))
            self.BR_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))
            self.BL_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))
        # elif self.joystick.getRawButton(9):
        #     if self.joystick.getLeftBumper():
        #         self.FR_steer_motor.set_control(self.position_voltage.with_position(desired_rotations*0.8))
        #         self.FL_steer_motor.set_control(self.position_voltage.with_position(desired_rotations*0.8))
        #         self.BR_steer_motor.set_control(self.position_voltage.with_position(-desired_rotations*0.8))
        #         self.BL_steer_motor.set_control(self.position_voltage.with_position(-desired_rotations*0.8))
        #     else:
        #         # Disable the motor instead
        #         self.FR_steer_motor.set_control(self.brake)
        #         self.FL_steer_motor.set_control(self.brake)
        #         self.BR_steer_motor.set_control(self.brake)
        #         self.BL_steer_motor.set_control(self.brake)  
        #         self.FR_drive_motor.set_control(self.brake)
        #         self.FL_drive_motor.set_control(self.brake)
        #         self.BR_drive_motor.set_control(self.brake)
        #         self.BL_drive_motor.set_control(self.brake)      
        else:
            # Disable the motor instead
            self.FR_steer_motor.set_control(self.brake)
            self.FL_steer_motor.set_control(self.brake)
            self.BR_steer_motor.set_control(self.brake)
            self.BL_steer_motor.set_control(self.brake)
            self.FR_drive_motor.set_control(self.brake)
            self.FL_drive_motor.set_control(self.brake)
            self.BR_drive_motor.set_control(self.brake)
            self.BL_drive_motor.set_control(self.brake)


        # TODO: control self.drive motor to reach "speed" (speed closed loop)

    
    # def 
    #     # Apply deadband
    #     if abs(x) < self.deadband_x:
    #         x = 0
    #     if abs(y) < self.deadband_y:
    #         y = 0
    #     if abs(rotation) < self.deadband_rotation:
    #         rotation = 0

    #     # Calculate velocities and angles
    #     velocity = math.hypot(x, y)
    #     angle = math.atan2(y, x) if x != 0 else 0

    #     # Limit velocity
    #     velocity = min(velocity, self.max_velocity)

    #     # Calculate motor speeds
    #     drive_speed, steer_speed = self.calculate_motor_speeds(velocity, angle)

    #     # Set motor speeds
    #     self.drive_motor.set(drive_speed)
    #     self.steer_motor.set(steer_speed)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    # def calculate_motor_speeds(self, velocity, angle):
    #     # Calculate drive and steer motor speeds using the velocity and angle
    #     drive_speed = velocity * math.cos(angle)
    #     steer_speed = velocity * math.sin(angle)

    #     # Limit motor acceleration
    #     drive_speed = max(-self.max_acceleration, min(self.max_acceleration, drive_speed))
    #     steer_speed = max(-self.max_acceleration, min(self.max_acceleration, steer_speed))

    #     return drive_speed, steer_speed

if __name__ == "__main__":
    wpilib.run(MyRobot)