#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpilib.shuffleboard
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
from phoenix6 import hardware, controls
import wpimath.units

kWheelRadius = 0.0508  # 轮子半径（单位：米）
kDriveMotorGearRatio = 8.14  # 驱动电机齿轮比
kTurningMotorGearRatio = 22.5  # 转向电机齿轮比
kEncoderResolution = 2048  # 编码器分辨率
kModuleMaxAngularVelocity = 180  # 模块最大角速度（单位：度/秒）
# kModuleMaxAngularAcceleration = math.tau/math.pi*180  # 模块最大角加速度（单位：度/秒^2）


class SwerveModule:
    def __init__(        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int,

    ) -> None:
        """构建一个SwerveModule，包含一个驱动电机、转向电机、和转向编码器。

        :param driveMotorChannel:      驱动电机的PWM输出。
        :param turningMotorChannel:    转向电机的PWM输出。
        :param turningEncoderChannel: 转向编码器通道的DIO输入
        """
        # self.driveMotor = wpilib.PWMTalonFX(driveMotorChannel)
        # self.turningMotor = wpilib.PWMTalonFX(turningMotorChannel)
        self.driveMotor = hardware.TalonFX(driveMotorChannel, "*")
        self.turningMotor = hardware.TalonFX(turningMotorChannel, "*")
        self.turningEncoder = hardware.CANcoder(turningEncoderChannel, "*")

        # self.turningMotor.set_position(self.turningEncoder.get_position().value)  ##### Check with lsy


        # Start at position 0, use slot 0 真正用到的PID cfg在这里传入
        self.position_voltage = controls.PositionVoltage(0).with_slot(0)
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        # Start at position 0, use slot 1
        # self.position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(1)

        # Gains are for example purposes only - must be determined for your own robot!
        # self.drivePIDController = wpimath.controller.PIDController(0.03, 0.000266394, 1.7) # Quote from 2022 - Jim Mei

        # Gains are for example purposes only - must be determined for your own robot!
        # self.turningPIDController = wpimath.controller.ProfiledPIDController(
        #     14, 0.003428576394, 420,  # This is degree PID from 2022!!!! - Jim Mei
        #     # 5, 0, 0,  ### PlaceHolder for Radian PID - Jim Mei
        #     wpimath.trajectory.TrapezoidProfile.Constraints(
        #         kModuleMaxAngularVelocity,
        #         kModuleMaxAngularAcceleration,
        #     ),
        # )

        #  No kF
        # # Gains are for example purposes only - must be determined for your own robot!
        # self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        # self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        # self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        # Add turning motor position to Shuffleboard # Shuffleboard 打印
        self.FL_turningMotor_pos = wpilib.shuffleboard.Shuffleboard.getTab("Swerve Module").add("FL Turning Motor Position", 0.1).getEntry()
        self.FR_turningMotor_pos = wpilib.shuffleboard.Shuffleboard.getTab("Swerve Module").add("FR Turning Motor Position", 0.1).getEntry()
        self.BL_turningMotor_pos = wpilib.shuffleboard.Shuffleboard.getTab("Swerve Module").add("BL Turning Motor Position", 0.1).getEntry()
        self.BR_turningMotor_pos = wpilib.shuffleboard.Shuffleboard.getTab("Swerve Module").add("BR Turning Motor Position", 0.1).getEntry()

        self.BR_desire_angle = wpilib.shuffleboard.Shuffleboard.getTab("Swerve Module").add("BR Desire Angle", 0.1).getEntry()

        self.current_rotation = wpilib.shuffleboard.Shuffleboard.getTab("Swerve Module").add("Current Rotation", 0.1).getEntry()

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module. 读取当前每组Swerve状态

        :returns: The current state of the module.
        """

        # Update Shuffleboard
        self.FL_turningMotor_pos.setDouble(self.turningMotor.get_position().value)
        self.FR_turningMotor_pos.setDouble(self.turningMotor.get_position().value)
        self.BL_turningMotor_pos.setDouble(self.turningMotor.get_position().value)
        self.BR_turningMotor_pos.setDouble(self.turningMotor.get_position().value)
        

        return wpimath.kinematics.SwerveModuleState(
            self.driveMotor.get_velocity()  / kDriveMotorGearRatio * kWheelRadius * math.pi * 2,   # *10
            wpimath.geometry.Rotation2d.fromDegrees(self.turningMotor.get_position().value / kTurningMotorGearRatio * 360)  # *10
        )

    # def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
    #     """Returns the current position of the module.

    #     :returns: The current position of the module.
    #     """
    #     return wpimath.kinematics.SwerveModulePosition(
    #         self.driveEncoder.getRate(),
    #         wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
    #     )

    def setDesiredState(        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module. 设置每组Swerve的期望状态

        :param desiredState: Desired state with speed and angle.
        """
        # if desiredState.speed == 0:  ######
        #     self.driveMotor.set_position(0)
        #     self.turningMotor.set_position(0)

        # encoderRotation = wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.turningMotor.get_position())) 
        # print(dir(self.turningMotor.get_position()))

        turningMotorPosition = self.turningMotor.get_position().value_as_double / kTurningMotorGearRatio * 360 ### To Degrees
        # print(f"Turning Motor Position: {turningMotorPosition}, Value: {turningMotorPosition.value}")

        currentRotation = wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(turningMotorPosition))   #### Check with lsy
        
        self.current_rotation.setDouble(currentRotation.degrees())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, currentRotation
        )

        # desiredAngle = state.angle.degrees()
        # angleDiff = self.turningMotor.get_position().value / kTurningMotorGearRatio * 360 - desiredAngle
        # desiredAngle += round((angleDiff) / 360) * 360

        desiredAngle = state.angle.degrees()
        angleDiff = self.turningMotor.get_position().value / kTurningMotorGearRatio * 360 - desiredAngle
        desiredAngle += round(angleDiff / 360) * 360
        # desiredAngle -= 180

        # Convert Desired Angle from degrees to rotations
        desiredAngle = desiredAngle * kTurningMotorGearRatio / 360
        # desiredAngle = 0

        # Update Shuffleboard
        self.BR_desire_angle.setDouble(desiredAngle)

        self.turningMotor.set_control(self.position_voltage.with_position(desiredAngle))
        velocity_voltage_value = self.velocity_voltage.with_velocity(state.speed/(kWheelRadius * math.pi * 2) * kDriveMotorGearRatio ) #/10
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