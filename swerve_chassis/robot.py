#!/usr/bin/env python3
"""
    This is a demo program for TalonFX Velocity PID usage in Phoenix 6
"""
import wpilib
import wpimath
import time
import math
import constants

from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds
from wpimath.filter import SlewRateLimiter
from wpilib import cameraserver
from swerve import Swerve, SwerveModule

from phoenix6 import hardware, signals, controls, configs, StatusCode



class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""
        
        self.swerve = Swerve()
        self.camera = cameraserver

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = SlewRateLimiter(3)
        self.yspeedLimiter = SlewRateLimiter(3)
        self.rotLimiter = SlewRateLimiter(3)

        # Keep a reference to all the motor controllers used
        # self.talonfx = hardware.TalonFX(0, canbusName)
        # self.talonfx_follower = hardware.TalonFX(1, canbusName)
        self.FRT_elevator_motor = hardware.TalonFX(14, "")
        self.BAK_elevator_motor = hardware.TalonFX(15, "")

        self.INT_intake_motor = hardware.TalonFX(16, "")
        self.LTP_intake_motor = hardware.TalonFX(22, "")

        self.TOP_shooter_motor = hardware.TalonFX(19, "")# update id
        self.BOT_shooter_motor = hardware.TalonFX(18, "")# update id
        self.ROT_shooter_motor = hardware.TalonFX(17, "")# update id
        self.RTP_shooter_motor = hardware.TalonFX(23, "")# update id

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
        self.position_voltage = controls.PositionVoltage(0).with_slot(0)
        # Start at position 0, use slot 1
        self.position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(1)
        # Keep a brake request so we can disable the motor
        self.brake = controls.NeutralOut()
        self.joystick = XboxController(0)



        cfg = configs.TalonFXConfiguration()
        # cfg.slot0.k_p = 3; # An error of 1 rotation results in 2.4 V output
        # cfg.slot0.k_i = 0; # No output for integrated error
        # cfg.slot0.k_d = 0.1; # A velocity of 1 rps results in 0.1 V output  # 0.1
        # # Peak output of 8 V

        # Swerve PID - 0818 new
        cfg.slot0.k_p = 0.25 # An error of 1 rotation results in 2.4 V output
        cfg.slot0.k_i = 0.0 # No output for integrated error
        cfg.slot0.k_d = 0.0 # A velocity of 1 rps results in 0.1 V output  # 0.1
        cfg.slot0.k_s = 0.1 # To account for friction, add 0.1 V of static feedforward
        # Peak output of 8 V
        cfg.slot0.k_v = 0.12
        cfg.voltage.peak_forward_voltage = 8
        cfg.voltage.peak_reverse_voltage = -8

        cfg.slot1.k_p = 60 # An error of 1 rotation results in 60 A output
        cfg.slot1.k_i = 0 # No output for integrated error
        cfg.slot1.k_d = 6 # A velocity of 1 rps results in 6 A output
        # Peak output of 120 A
        cfg.torque_current.peak_forward_torque_current = 120
        cfg.torque_current.peak_reverse_torque_current = -120

        cfg_drive = configs.TalonFXConfiguration()
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
            

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
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

        
        # self.FR_drive_motor.set_control(controls.Follower(self.FR_steer_motor.device_id, False))   ### TBD!!!!
        # self.FL_drive_motor.set_control(controls.Follower(self.FL_steer_motor.device_id, False))
        # self.BR_drive_motor.set_control(controls.Follower(self.BR_steer_motor.device_id, False))
        # self.BL_drive_motor.set_control(controls.Follower(self.BL_steer_motor.device_id, False))

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # joy_value = self.joystick.getLeftX()
        # if abs(joy_value) < 0.1:
        #     joy_value = 0

        # Go for plus/minus 50 rotations per second
    


############################################### New Swerve Drive ########################################################
        forwardAxis = - self.joystick.getLeftY()
        xSpeed = self.xspeedLimiter.calculate(wpimath.applyDeadband(forwardAxis, 0.1)) * 1.5  ## 0.1 is the deadband; 2.5 is the max speed - 0818new

        strafeAxis = - self.joystick.getLeftX()
        ySpeed = self.yspeedLimiter.calculate(wpimath.applyDeadband(strafeAxis, 0.1)) * 1.5  ## 0.1 is the deadband; 2.5 is the max speed - 0818new

        # yawX = - self.joystick.getRightX()
        # yawY = - self.joystick.getRightY()
        # targetYaw = math.atan2(yawX, yawY)

        # if yawX > 0 and yawY > 0:
        #     targetYaw = -targetYaw
        # elif yawX > 0 and yawY < 0:
        #     targetYaw = -180 - targetYaw
        # elif yawX < 0 and yawY < 0:
        #     targetYaw = 180 - targetYaw
        # elif yawX < 0 and yawY > 0:
        #     targetYaw = -targetYaw

        # yawAxis = math.sqrt(yawX*yawX + yawY*yawY)
        # rot = self.rotLimiter.calculate(wpimath.applyDeadband(yawAxis, 0.1)) * 1  ## 0.1 is the deadband; 1 is the max turning speed - 0818new Help Me!!!!!
        rotAxis = - self.joystick.getRightX()
        rot = self.rotLimiter.calculate(wpimath.applyDeadband(rotAxis, 0.1)) * 1  ## 0.1 is the deadband; 1 is the max turning speed - 0818new Help Me!!!!!
        self.swerve.drive(xSpeed, ySpeed, rot)
          ### -0818new Help Me!!!!

        
    def testInit(self):
        pass

    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)