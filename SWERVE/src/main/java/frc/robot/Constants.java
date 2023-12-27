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

  public static class OIConstants {

    public static double kDeadband = 0.05;

    public static int driverControllerPort = 0;

  }

  public static class DriveTrainConstants {

    // max acceleration desired
    public static double kDriveMaxAcceleration = 2.5;
    public static double kDriveMaxAngularAcceleration = 180;

    // pid to be tuned later
    public static double kPTurning = 1;
    public static double kITurning = 0;
    public static double kDTurning = 0;
    public static double kPDrive = 1;
    public static double kIDrive = 0;
    public static double kDDrive = 0;

    // values for each motor to be tested and recorded
    public static boolean topLeftAbsoluteEncoderReversed = false;
    public static double topLeftAbsoluteEncoderOffset = 0;
    public static boolean topLeftDriveMotorReversed = false;
    public static boolean topLeftTurnMotorReversed = false;

    public static boolean topRightAbsoluteEncoderReversed = false;
    public static double topRightAbsoluteEncoderOffset = 0;
    public static boolean topRightDriveMotorReversed = false;
    public static boolean topRightTurnMotorReversed = false;

    public static boolean bottomLeftAbsoluteEncoderReversed = false;
    public static double bottomLeftAbsoluteEncoderOffset = 0;
    public static boolean bottomLeftDriveMotorReversed = false;
    public static boolean bottomLeftTurnMotorReversed = false;

    public static boolean bottomRightAbsoluteEncoderReversed = false;
    public static double bottomRightAbsoluteEncoderOffset = 0;
    public static boolean bottomRightDriveMotorReversed = false;
    public static boolean bottomRightTurnMotorReversed = false;
  
    // ids for each motor
    public static int topLeftDriveMotorID = 1;
    public static int topRightDriveMotorID = 2;
    public static int bottomLeftDriveMotorID = 3;
    public static int bottomRightDriveMotorID = 4;

    public static int topLeftTurnMotorID = 5;
    public static int topRightTurnMotorID = 6;
    public static int bottomLeftTurnMotorID = 7;
    public static int bottomRightTurnMotorID = 8;

    // conversion factors: multiply encoder output by these factors to get a different unit
    public static double turnEncoderToRad = 2 * Math.PI /4096;
    public static double driveEncoderToMeters = 1 / 2048 * Math.PI * 2; // * wheel radius * gear ratio
  
    // gyroscope
    public static int gyroPort = 1;
  
    // max speed
    public static double physicalMaxSpeed = 5.0;

    // physical info for kinematics (change to real info)
    public static final double trackWidth = 20;
    public static final double wheelBase = 20;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase/2, -trackWidth/2),
      new Translation2d(wheelBase/2, trackWidth/2),
      new Translation2d(-wheelBase/2, -trackWidth/2),
      new Translation2d(-wheelBase/2, trackWidth/2)
    );
  }

}
