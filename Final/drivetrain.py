#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule

from wpilib.shuffleboard import Shuffleboard

kMaxSpeed = 2.0  # 3 meters per second   最大底盘线速度
kMaxAngularSpeed = 180  # 1/2 rotation per second   最大底盘转向速度


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:

        trackwidthMeters = 0.52705 ### 横向轮距
        wheelbaseMeters = 0.52705  ### 纵向轮距

        # 找中心点
        self.frontLeftLocation = wpimath.geometry.Translation2d(trackwidthMeters/2, wheelbaseMeters/2)
        self.frontRightLocation = wpimath.geometry.Translation2d(trackwidthMeters/2, -wheelbaseMeters/2)
        self.backLeftLocation = wpimath.geometry.Translation2d(-trackwidthMeters/2, wheelbaseMeters/2)
        self.backRightLocation = wpimath.geometry.Translation2d(-trackwidthMeters/2, -wheelbaseMeters/2)
        
        # self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        # self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        # self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        # self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        # 初始化四个对应Sweve模块
        self.frontLeft = swervemodule.SwerveModule(driveMotorChannel=0, turningMotorChannel=1, turningEncoderChannel=20)
        self.frontRight = swervemodule.SwerveModule(driveMotorChannel=10, turningMotorChannel=12, turningEncoderChannel=21)
        self.backLeft = swervemodule.SwerveModule(driveMotorChannel=3, turningMotorChannel=4, turningEncoderChannel=5)
        self.backRight = swervemodule.SwerveModule(driveMotorChannel=6, turningMotorChannel=7, turningEncoderChannel=8)

        # 初始化Gyro， 我们没用到
        self.gyro = wpilib.AnalogGyro(0) # TODO: phoenix6.hardware.corepegion2

        # 初始化底盘运动学（Kinematics）成员
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        # self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
        #     self.kinematics,
        #     self.gyro.getRotation2d(),
        #     (
        #         self.frontLeft.getPosition(),
        #         self.frontRight.getPosition(),00
        #         self.backLeft.getPosition(),
        #         self.backRight.getPosition(),
        #     ),
        # )

        # 重置Gyro
        self.gyro.reset()

        # add shuffleboard tab Shuffleboard打印初始化语句
        self.FL_speed = Shuffleboard.getTab("Swerve").add("Front Left Speed", 0.1).getEntry() # set to 0.1
        self.FR_speed = Shuffleboard.getTab("Swerve").add("Front Right Speed", 0.1).getEntry()
        self.BL_speed = Shuffleboard.getTab("Swerve").add("Back Left Speed", 0.1).getEntry()
        self.BR_speed = Shuffleboard.getTab("Swerve").add("Back Right Speed", 0.1).getEntry()

        # add angle to Shuffle  Board
        self.FL_angle = Shuffleboard.getTab("Swerve").add("Front Left Angle", 0.1).getEntry()
        self.FR_angle = Shuffleboard.getTab("Swerve").add("Front Right Angle", 0.1).getEntry()
        self.BL_angle = Shuffleboard.getTab("Swerve").add("Back Left Angle", 0.1).getEntry()
        self.BR_angle = Shuffleboard.getTab("Swerve").add("Back Right Angle", 0.1).getEntry()

        self.BR_actual_angle = Shuffleboard.getTab("Swerve").add("BRActual Angle", 0.1).getEntry()

        # self.BR_desire_angle = Shuffleboard.getTab("Swerve").add("BRDesire Angle", 0.1).getEntry()
    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
    ) -> None:
        """
        使用摇杆输入值驱动机器人的方法。  真正最后在robot.py中调用的方法  ！！！！
        
        :param xSpeed: 机器人在x方向（前进）的速度。
        :param ySpeed: 机器人在y方向（侧向）的速度。
        :param rot: 机器人的角速度。
        :param fieldRelative: 提供的x和y速度是否相对于场地。
        :param periodSeconds: 时间
        
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        # 读取Swerve模块的状态
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot) 
        )
        # 利用wpilib的方法限制过饱和底盘速度 （减少抽搐用的）
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        
        # 设置期望状态
        self.frontLeft.setDesiredState(swerveModuleStates[0])   
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

        # Update Shuffleboard
        self.FL_speed.setDouble(swerveModuleStates[0].speed)
        self.FR_speed.setDouble(swerveModuleStates[1].speed)
        self.BL_speed.setDouble(swerveModuleStates[2].speed)
        self.BR_speed.setDouble(swerveModuleStates[3].speed)

        self.FL_angle.setDouble(swerveModuleStates[0].angle.degrees()) 
        self.FR_angle.setDouble(swerveModuleStates[1].angle.degrees())
        self.BL_angle.setDouble(swerveModuleStates[2].angle.degrees())
        self.BR_angle.setDouble(swerveModuleStates[3].angle.degrees())

        self.BR_actual_angle.setDouble(self.backRight.turningMotor.get_position().value / swervemodule.kTurningMotorGearRatio * 360)
        
        

    # def updateOdometry(self) -> None:
    #     """Updates the field relative position of the robot."""
    #     self.odometry.update(
    #         self.gyro.getRotation2d(),
    #         (
    #             self.frontLeft.getPosition(),
    #             self.frontRight.getPosition(),
    #             self.backLeft.getPosition(),
    #             self.backRight.getPosition(),
    #         ),
    #     )