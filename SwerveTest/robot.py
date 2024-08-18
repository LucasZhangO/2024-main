#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
import phoenix6

from phoenix6 import hardware


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()
    
        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)
        self.FR_steer_motor = hardware.TalonFX(12, "*")
        self.FR_drive_motor = hardware.TalonFX(10, "*")
        self.FL_steer_motor = hardware.TalonFX(1, "*")
        self.FL_drive_motor = hardware.TalonFX(0, "*")
        self.BR_steer_motor = hardware.TalonFX(7, "*")
        self.BR_drive_motor = hardware.TalonFX(6, "*")
        self.BL_steer_motor = hardware.TalonFX(4, "*")
        self.BL_drive_motor = hardware.TalonFX(3, "*")


        cfg = phoenix6.configs.TalonFXConfiguration()
        # Swerve PID - 0818 new
        cfg.slot0.k_p = 0.25 # An error of 1 rotation results in 2.4 V output
        cfg.slot0.k_i = 0.0 # No output for integrated error
        cfg.slot0.k_d = 0.0 # A velocity of 1 rps results in 0.1 V output  # 0.1
        cfg.slot0.k_s = 0.1 # To account for friction, add 0.1 V of static feedforward
        # Peak output of 8 V
        cfg.slot0.k_v = 0.12
        # cfg.slot0.k_v = 0.05
        cfg.voltage.peak_forward_voltage = 8
        cfg.voltage.peak_reverse_voltage = -8


        cfg_drive = phoenix6.configs.TalonFXConfiguration()
        # cfg.slot0.k_p = 3; # An error of 1 rotation results in 2.4 V output
        # cfg.slot0.k_i = 0; # No output for integrated error
        # cfg.slot0.k_d = 0.1; # A velocity of 1 rps results in 0.1 V output  # 0.1
        # # Peak output of 8 V

        # Swerve PID - 0818 new
        cfg_drive.slot0.k_p = 0.2 # An error of 1 rotation results in 2.4 V output
        cfg_drive.slot0.k_i = 0.005 # No output for integrated error
        cfg_drive.slot0.k_d = 0.0 # A velocity of 1 rps results in 0.1 V output  # 0.1
        # Peak output of 8 V
        cfg_drive.slot0.k_v = 0.12
        cfg_drive.voltage.peak_forward_voltage = 8
        cfg_drive.voltage.peak_reverse_voltage = -8

        status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED

        for _ in range(0, 5):
            status = self.FR_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        # self.FR_steer_motor.set_position(0)

        for _ in range(0, 5):
            status = self.FL_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        # Make sure we start at 0
        # self.FL_steer_motor.set_position(0)

        
        for _ in range(0, 5):
            status = self.BR_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        # self.BR_steer_motor.set_position(0)

        
        for _ in range(0, 5):
            status = self.BL_steer_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        # self.BL_steer_motor.set_position(0)
            

        status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.FR_drive_motor.configurator.apply(cfg_drive)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        for _ in range(0, 5):
            status = self.FL_drive_motor.configurator.apply(cfg_drive)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        for _ in range(0, 5):
            status = self.BR_drive_motor.configurator.apply(cfg_drive)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
            
        for _ in range(0, 5):
            status = self.BL_drive_motor.configurator.apply(cfg_drive)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick()

    def driveWithJoystick(self) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            - self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            - self.rotLimiter.calculate(
                wpimath.applyDeadband( self.controller.getRightX(), 0.1)
            )
            * drivetrain.kMaxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)