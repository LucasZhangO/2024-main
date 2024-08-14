#!/usr/bin/env python3
"""
    This is a demo program for TalonFX Velocity PID usage in Phoenix 6
"""
import wpilib
import math


from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds

from phoenix6 import hardware, signals, controls, configs, StatusCode

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        # self.talonfx = hardware.TalonFX(0, canbusName)
        # self.talonfx_follower = hardware.TalonFX(1, canbusName)
        self.FR_steer_motor = hardware.TalonFX(12, "*")
        self.FR_drive_motor = hardware.TalonFX(10, "*")
        self.FL_steer_motor = hardware.TalonFX(1, "*")
        self.FL_drive_motor = hardware.TalonFX(0, "*")
        self.BR_steer_motor = hardware.TalonFX(7, "*")
        self.BR_drive_motor = hardware.TalonFX(6, "*")
        self.BL_steer_motor = hardware.TalonFX(4, "*")
        self.BL_drive_motor = hardware.TalonFX(3, "*")
        # Be able to switch which control request to use based on a button press
        # Start at velocity 0, use slot 0
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        # Start at velocity 0, use slot 1
        self.velocity_torque = controls.VelocityTorqueCurrentFOC(0).with_slot(1)
        XboxController = wpilib.XboxController
        self.joystick = XboxController(0)
        # Keep a neutral out so we can disable the motor
        self.brake = controls.NeutralOut()

        self.joystick = XboxController(0)

        cfg = configs.TalonFXConfiguration()
        
        # Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
        cfg.slot0.k_s = 0.1 # To account for friction, add 0.1 V of static feedforward
        cfg.slot0.k_v = 0.12 # Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        cfg.slot0.k_p = 0.11 # An error of 1 rotation per second results in 2V output
        cfg.slot0.k_i = 0 # No output for integrated error
        cfg.slot0.k_d = 0 # No output for error derivative
        # Peak output of 8 volts
        cfg.voltage.peak_forward_voltage = 8
        cfg.voltage.peak_reverse_voltage = -8

        # Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself
        cfg.slot1.k_s = 2.5 # To account for friction, add 2.5 A of static feedforward
        cfg.slot1.k_p = 5 # An error of 1 rotation per second results in 5 A output
        cfg.slot1.k_i = 0 # No output for integrated error
        cfg.slot1.k_d = 0 # No output for error derivative
        # Peak output of 40 A
        cfg.torque_current.peak_forward_torque_current = 40
        cfg.torque_current.peak_reverse_torque_current = -40

        # Retry config apply up to 5 times, report if failure
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
    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        joy_value = self.joystick.getLeftX()
        if abs(joy_value) < 0.1:
            joy_value = 0

        # Go for plus/minus 50 rotations per second
        desired_rotations_per_second = joy_value * 50

        if self.joystick.getLeftBumper():
            # Use velocity voltage
            self.FR_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
            self.FL_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
            self.BR_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
            self.BL_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
        else:
            # Disable the motor instead
            self.FR_drive_motor.set_control(self.brake)
            self.FL_drive_motor.set_control(self.brake)
            self.BR_drive_motor.set_control(self.brake)
            self.BL_drive_motor.set_control(self.brake)
        
    def testInit(self):
        pass

    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)