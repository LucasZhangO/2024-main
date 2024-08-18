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

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:

        trackwidthMeters = 0.52705
        wheelbaseMeters = 0.52705

        self.frontLeftLocation = wpimath.geometry.Translation2d(trackwidthMeters/2, wheelbaseMeters/2)
        self.frontRightLocation = wpimath.geometry.Translation2d(trackwidthMeters/2, -wheelbaseMeters/2)
        self.backLeftLocation = wpimath.geometry.Translation2d(-trackwidthMeters/2, wheelbaseMeters/2)
        self.backRightLocation = wpimath.geometry.Translation2d(-trackwidthMeters/2, -wheelbaseMeters/2)
        
        # self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        # self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        # self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        # self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        self.frontLeft = swervemodule.SwerveModule(driveMotorChannel=0, turningMotorChannel=1, turningEncoderChannelA=20)
        self.frontRight = swervemodule.SwerveModule(driveMotorChannel=10, turningMotorChannel=12, turningEncoderChannelA=21)
        self.backLeft = swervemodule.SwerveModule(driveMotorChannel=3, turningMotorChannel=4, turningEncoderChannelA=5)
        self.backRight = swervemodule.SwerveModule(driveMotorChannel=6, turningMotorChannel=7, turningEncoderChannelA=8)

        self.gyro = wpilib.AnalogGyro(0)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                    if fieldRelative
                    else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )