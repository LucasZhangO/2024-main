#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
from phoenix6 import hardware, controls
import wpimath.units

kWheelRadius = 0.0508
kDriveMotorGearRatio = 8.14
kTurningMotorGearRatio = 14.2857
kEncoderResolution = 2048
kModuleMaxAngularVelocity = 180
kModuleMaxAngularAcceleration = math.tau/math.pi*180


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int,

    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        # :param driveEncoderChannelA:   DIO input for the drive encoder channel A
        # :param driveEncoderChannelB:   DIO input for the drive encoder channel B
        :param turningEncoderChannelA: DIO input for the turning encoder channel A
        :param turningEncoderChannelB: DIO input for the turning encoder channel B
        """
        # self.driveMotor = wpilib.PWMTalonFX(driveMotorChannel)
        # self.turningMotor = wpilib.PWMTalonFX(turningMotorChannel)
        self.driveMotor = hardware.TalonFX(driveMotorChannel, "*")
        self.turningMotor = hardware.TalonFX(turningMotorChannel, "*")
        self.turningEncoder = hardware.CANcoder(turningEncoderChannel, "*")

        self.turningMotor.set_position(self.turningEncoder.get_position().value)  ##### Check with lsy


        # Start at position 0, use slot 0
        self.position_voltage = controls.PositionVoltage(0).with_slot(0)
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        # Start at position 0, use slot 1
        self.position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(1)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(0.03, 0.000266394, 1.7) # Quote from 2022 - Jim Mei

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            14, 0.003428576394, 420,  # This is degree PID from 2022!!!! - Jim Mei
            # 5, 0, 0,  ### PlaceHolder for Radian PID - Jim Mei
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        #  No kF
        # # Gains are for example purposes only - must be determined for your own robot!
        # self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        # self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveMotor.get_velocity() * 10 / kDriveMotorGearRatio * kWheelRadius * math.pi * 2, 
            wpimath.geometry.Rotation2d.fromDegrees(self.turningMotor.get_position().value),
        )

    # def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
    #     """Returns the current position of the module.

    #     :returns: The current position of the module.
    #     """
    #     return wpimath.kinematics.SwerveModulePosition(
    #         self.driveEncoder.getRate(),
    #         wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
    #     )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        # if desiredState.speed == 0:  ######
        #     self.driveMotor.set_position(0)
        #     self.turningMotor.set_position(0)

        # encoderRotation = wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.turningMotor.get_position())) 
        # print(dir(self.turningMotor.get_position()))

        turningMotorPosition = self.turningMotor.get_position().value
        # print(f"Turning Motor Position: {turningMotorPosition}, Value: {turningMotorPosition.value}")

        encoderRotation = wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(turningMotorPosition))   #### Check with lsy
        

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        desiredAngle = state.angle.degrees()
        angleDiff = self.turningMotor.get_position().value - desiredAngle
        desiredAngle += round(angleDiff / 360) * 360

        # Convert Desired Angle from degrees to rotations
        desiredAngle = desiredAngle / 360

        self.turningMotor.set_control(self.position_voltage.with_position(desiredAngle))
        velocity_voltage_value = self.velocity_voltage.with_velocity(state.speed/(kWheelRadius * math.pi * 2) * kDriveMotorGearRatio / 10)
        self.driveMotor.set_control(velocity_voltage_value)


        # # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # # direction of travel that can occur when modules change directions. This results in smoother
        # # driving.
        # state.speed *= (state.angle - encoderRotation).cos()

        # # Calculate the drive output from the drive PID controller.
        # driveOutput = self.drivePIDController.calculate(
        #     self.driveEncoder.getRate(), state.speed
        # )

        # driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # # Calculate the turning motor output from the turning PID controller.
        # turnOutput = self.turningPIDController.calculate(
        #     self.turningEncoder.getDistance(), state.angle.radians()
        # )

        # turnFeedforward = self.turnFeedforward.calculate(
        #     self.turningPIDController.getSetpoint().velocity
        # )

        # self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        # self.turningMotor.setVoltage(turnOutput + turnFeedforward)