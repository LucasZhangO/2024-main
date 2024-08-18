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
        



        cfg_e = configs.TalonFXConfiguration()
        
        # Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
        cfg_e.slot0.k_s = 0.1 # To account for friction, add 0.1 V of static feedforward
        cfg_e.slot0.k_v = 0.12 # Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        cfg_e.slot0.k_p = 0.11 # An error of 1 rotation per second results in 2V output
        cfg_e.slot0.k_i = 0 # No output for integrated error
        cfg_e.slot0.k_d = 0 # No output for error derivative
        # Peak output of 8 volts
        cfg_e.voltage.peak_forward_voltage = 8
        cfg_e.voltage.peak_reverse_voltage = -8

        # Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself
        cfg_e.slot1.k_s = 2.5 # To account for friction, add 2.5 A of static feedforward
        cfg_e.slot1.k_p = 5 # An error of 1 rotation per second results in 5 A output
        cfg_e.slot1.k_i = 0 # No output for integrated error
        cfg_e.slot1.k_d = 0 # No output for error derivative
        # Peak output of 40 A
        cfg_e.torque_current.peak_forward_torque_current = 40
        cfg_e.torque_current.peak_reverse_torque_current = -40

        cfg_i = configs.TalonFXConfiguration()
        
        # Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
        cfg_i.slot0.k_s = 0 # To account for friction, add 0.1 V of static feedforward
        cfg_i.slot0.k_v = 0.11999999731779099 # Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        cfg_i.slot0.k_p = 0.30000001192092896 # An error of 1 rotation per second results in 2V output
        cfg_i.slot0.k_i = 0.05000000074505806 # No output for integrated error
        cfg_i.slot0.k_d = 0 # No output for error derivative
        # Peak output of 8 volts
        cfg_i.voltage.peak_forward_voltage = 8
        cfg_i.voltage.peak_reverse_voltage = -8

        # Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself
        cfg_i.slot1.k_s = 2.5 # To account for friction, add 2.5 A of static feedforward
        cfg_i.slot1.k_p = 5 # An error of 1 rotation per second results in 5 A output
        cfg_i.slot1.k_i = 0 # No output for integrated error
        cfg_i.slot1.k_d = 0 # No output for error derivative
        # Peak output of 40 A
        cfg_i.torque_current.peak_forward_torque_current = 40
        cfg_i.torque_current.peak_reverse_torque_current = -40

        cfg_s = configs.TalonFXConfiguration()
        
        # Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
        cfg_s.slot0.k_s = 0.1 # To account for friction, add 0.1 V of static feedforward
        cfg_s.slot0.k_v = 0.12 # Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        cfg_s.slot0.k_p = 0.11 # An error of 1 rotation per second results in 2V output
        cfg_s.slot0.k_i = 0 # No output for integrated error
        cfg_s.slot0.k_d = 0 # No output for error derivative
        # Peak output of 8 volts
        cfg_s.voltage.peak_forward_voltage = 8
        cfg_s.voltage.peak_reverse_voltage = -8

        # Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself
        cfg_s.slot1.k_s = 2.5 # To account for friction, add 2.5 A of static feedforward
        cfg_s.slot1.k_p = 5 # An error of 1 rotation per second results in 5 A output
        cfg_s.slot1.k_i = 0 # No output for integrated error
        cfg_s.slot1.k_d = 0 # No output for error derivative
        # Peak output of 40 A
        cfg_s.torque_current.peak_forward_torque_current = 40
        cfg_s.torque_current.peak_reverse_torque_current = -40


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


        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.FRT_elevator_motor.configurator.apply(cfg_e)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        for _ in range(0, 5):
            status = self.BAK_elevator_motor.configurator.apply(cfg_e)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

            
        for _ in range(0, 5):
            status = self.INT_intake_motor.configurator.apply(cfg_i)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        for _ in range(0, 5):
            status = self.LTP_intake_motor.configurator.apply(cfg_i)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")


        for _ in range(0, 5):
            status = self.TOP_shooter_motor.configurator.apply(cfg_s)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        for _ in range(0, 5):
            status = self.BOT_shooter_motor.configurator.apply(cfg_s)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        for _ in range(0, 5):
            status = self.ROT_shooter_motor.configurator.apply(cfg_s) # for shooter can be modify
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        for _ in range(0, 5):
            status = self.RTP_shooter_motor.configurator.apply(cfg_s)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
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
    
############################################## Elevator ########################################################
        if self.joystick.getRightBumper():#
            # Use velocity voltage
            self.FRT_elevator_motor.set_control(self.velocity_voltage.with_velocity(30))# update 
            self.BAK_elevator_motor.set_control(self.velocity_voltage.with_velocity(30))# update max valume

        
        elif self.joystick.getLeftBumper():#
            # Use velocity voltage
            self.FRT_elevator_motor.set_control(self.velocity_voltage.with_velocity(-30))# update 
            self.BAK_elevator_motor.set_control(self.velocity_voltage.with_velocity(-30))# update max valume

        else:
            self.FRT_elevator_motor.set_control(self.brake)
            self.BAK_elevator_motor.set_control(self.brake)
################################################################################################################

############################################## Intake ##########################################################
        if self.joystick.getAButton():#
            # Use velocity voltage
            self.INT_intake_motor.set_control(self.velocity_voltage.with_velocity(30))# update 
            self.LTP_intake_motor.set_control(self.velocity_voltage.with_velocity(-30))# update
        
        elif self.joystick.getYButton():#
            # Use velocity voltage
            self.INT_intake_motor.set_control(self.velocity_voltage.with_velocity(30))# update 
            self.LTP_intake_motor.set_control(self.velocity_voltage.with_velocity(30))# update
        else:
            # Disable the motor instead
            self.INT_intake_motor.set_control(self.brake)
            self.LTP_intake_motor.set_control(self.brake)
################################################################################################################
        
        # else:
        #     self.BL_steer_motor.set_position(0)
        #     self.BR_steer_motor.set_position(0)
        #     self.FL_steer_motor.set_position(0)
        #     self.FR_steer_motor.set_control(0)

        
########################################### Shooter ##############################################################
        if self.joystick.getRightTriggerAxis():#
            # Use velocity voltage
            self.TOP_shooter_motor.set_control(self.velocity_voltage.with_velocity(40))# update 
            self.BOT_shooter_motor.set_control(self.velocity_voltage.with_velocity(-40))# update max valume
            time.sleep(0.5)   #### Tunr the time
            self.RTP_shooter_motor.set_control(self.velocity_voltage.with_velocity(-30))
        else:
            self.TOP_shooter_motor.set_control(self.brake)
            self.BOT_shooter_motor.set_control(self.brake)
            self.RTP_shooter_motor.set_control(self.brake)
####################################################################################################################

############################################### New Swerve Drive ########################################################
        forwardAxis = - self.joystick.getLeftY()
        xSpeed = self.xspeedLimiter.calculate(wpimath.applyDeadband(forwardAxis, 0.1)) * 1  ## 0.1 is the deadband; 2.5 is the max speed - 0818new

        strafeAxis = - self.joystick.getLeftX()
        ySpeed = self.yspeedLimiter.calculate(wpimath.applyDeadband(strafeAxis, 0.1)) * 1  ## 0.1 is the deadband; 2.5 is the max speed - 0818new

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
        rotAxis = -self.joystick.getRightX()
        rot = self.rotLimiter.calculate(wpimath.applyDeadband(rotAxis, 0.1)) * 1  ## 0.1 is the deadband; 1 is the max turning speed - 0818new Help Me!!!!!
        self.swerve.drive(xSpeed, ySpeed, rot)
          ### -0818new Help Me!!!!
############################################## Old Swerve Drive ########################################################
        # desired_rotations_per_second = joy_value * 50
        # desired_rotations = self.joystick.getLeftY() * 10
        # if abs(desired_rotations) <= 0.1: # Joystick deadzone
        #     desired_rotations = 0

        # if self.joystick.getXButton():
        #     self.BL_steer_motor.set_position(0)
        #     self.BR_steer_motor.set_position(0)
        #     self.FL_steer_motor.set_position(0)
        #     self.FR_steer_motor.set_position(0)

        # if self.joystick.getLeftStickButton():
        #     # Use position voltage
        #     self.FR_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
        #     self.FL_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
        #     self.BR_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
        #     self.BL_drive_motor.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
        #     self.FR_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))
        #     self.FL_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))
        #     self.BR_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))
        #     self.BL_steer_motor.set_control(self.position_voltage.with_position(desired_rotations))

        # elif self.joystick.getRightStickButtonPressed():   # 9
        #     if self.joystick.getRightStickButton():
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
        # else:
        #     # Disable the motor instead
        #     self.FR_steer_motor.set_control(self.brake)
        #     self.FL_steer_motor.set_control(self.brake)
        #     self.BR_steer_motor.set_control(self.brake)
        #     self.BL_steer_motor.set_control(self.brake)
        #     self.FR_drive_motor.set_control(self.brake)
        #     self.FL_drive_motor.set_control(self.brake)
        #     self.BR_drive_motor.set_control(self.brake)
        #     self.BL_drive_motor.set_control(self.brake)
#########################################################################################################################
        
        # shooter rotation adjust 
        # rot_value=self.joystick.getRightY()
        # if abs(rot_value) < 0.1:
        #     rot_value = 0
        # if self.joystick.getRightStickButton():
        #     # Use velocity voltage
        #     self.ROT_shooter_motor.set_control(self.velocity_voltage.with_acceleration(rot_value))# update 
        # else:
        #     # Disable the motor instead
        #     self.ROT_shooter_motor.set_control(self.brake)


        
    def testInit(self):
        pass

    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)