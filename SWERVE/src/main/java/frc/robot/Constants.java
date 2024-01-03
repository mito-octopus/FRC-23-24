// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OIConstants { // input/output constants
    public static double kDeadband = 0.05;
    public static int kControllerPort = 0;
  }

  public static class DriveTrainConstants {

    // pid for the swerve modules
    public static double kPTurning = 1;
    public static double kITurning = 0;
    public static double kDTurning = 0;

    public static double kPDrive = 1;
    public static double kIDrive = 0;
    public static double kDDrive = 0;

    // pid for the trajectory following
    public static final double kPx = 1.0;
    public static final double kIx = 0;
    public static final double kDx = 0;

    public static final double kPy = 1.0;
    public static final double kIy = 0;
    public static final double kDy = 0;

    public static final double kPr = 1.0;
    public static final double kIr = 0;
    public static final double kDr = 0;

    // maximum desired chassis acceleration and velocity
    public static double kDriveMaxAcceleration = 2.5;
    public static double kDriveMaxVelocity = 5.0;

    public static double kDriveMaxAngularAccelerationDegrees = 180;
    public static final double kMaxAngularAccelerationRad = Math.PI/2;
    public static final double kMaxAngularVelocityRad = 6.28;

    // swerve module intialization information
    public static int kFrontLeftDriveMotorID = 1;
    public static int kFrontLeftTurnMotorID = 5;
    public static boolean kFrontLeftAbsoluteEncoderReversed = false;
    public static double kFrontLeftAbsoluteEncoderOffset = 0;
    public static boolean kFrontLeftDriveMotorReversed = false;
    public static boolean kFrontLeftTurnMotorReversed = false;

    public static int kFrontRightDriveMotorID = 2;
    public static int kFrontRightTurnMotorID = 6;
    public static boolean kFrontRightAbsoluteEncoderReversed = false;
    public static double kFrontRightAbsoluteEncoderOffset = 0;
    public static boolean kFrontRightDriveMotorReversed = false;
    public static boolean kFrontRightTurnMotorReversed = false;

    public static int kBackLeftDriveMotorID = 3;
    public static int kBackLeftTurnMotorID = 7;
    public static boolean kBackLeftAbsoluteEncoderReversed = false;
    public static double kBackLeftAbsoluteEncoderOffset = 0;
    public static boolean kBackLeftDriveMotorReversed = false;
    public static boolean kBackLeftTurnMotorReversed = false;

    public static int kBackRightDriveMotorID = 4;
    public static int kBackRightTurnMotorID = 8;
    public static boolean kBackRightAbsoluteEncoderReversed = false;
    public static double kBackRightAbsoluteEncoderOffset = 0;
    public static boolean kBackRightDriveMotorReversed = false;
    public static boolean kBackRightTurnMotorReversed = false;
  
    // encoder conversion factors
    public static double kTurnEncoderToRad = 2 * Math.PI /4096;
    public static double kDriveEncoderToMeters = 1 / 2048 * Math.PI * 2; // * wheel radius * gear ratio
  
    // gyroscope port
    public static int kGyroPort = 1;
  
    // physical info for kinematics
    public static final double kTrackWidth = 20;
    public static final double kWheelBase = 20;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase/2, -kTrackWidth/2),
      new Translation2d(kWheelBase/2, kTrackWidth/2),
      new Translation2d(-kWheelBase/2, -kTrackWidth/2),
      new Translation2d(-kWheelBase/2, kTrackWidth/2)
    );
  }

}
