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
        self.INT_intake_motor = hardware.TalonFX(16, "*")# update id

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

        cfg_i = configs.TalonFXConfiguration()
        
        # Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
        cfg_i.slot0.k_s = 0.1 # To account for friction, add 0.1 V of static feedforward
        cfg_i.slot0.k_v = 0.12 # Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        cfg_i.slot0.k_p = 0.11 # An error of 1 rotation per second results in 2V output
        cfg_i.slot0.k_i = 0 # No output for integrated error
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

        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.INT_intake_motor.configurator.apply(cfg_i)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        



    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        joy_value = self.joystick.getLeftX()
        if abs(joy_value) < 0.1:
            joy_value = 0

        # Go for plus/minus 50 rotations per second
    

        if self.joystick.getBButton():#
            # Use velocity voltage
            self.INT_intake_motor.set_control(self.velocity_voltage.with_velocity(30))# update 
            
        else:
            # Disable the motor instead
            self.INT_intake_motor.set_control(self.brake)
    def testInit(self):
        pass

    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)