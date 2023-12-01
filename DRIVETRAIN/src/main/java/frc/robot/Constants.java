// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveTrainConstants {
    public static final int kLeftTalonID1 = 0;
    public static final int kLeftTalonID2 = 1;
    public static final int kRightTalonID1 = 2;
    public static final int kRightTalonID2 = 3;


    public static final double kMaxAccelerationUnitsPerSecond = 3;
    // this is basically a factor that, when multiplied by the integrated falcon sensor reading, will give 
    // an indication of how many meters that wheel has turned (r * 2 * pi = the circumerence / 2048, which is the number of ticks outputted for a full rotation)
    public static final double sensorFactor = 0.3 * 2.0 * Math.PI / 2048.0;  // CHANGE THE FIRST VALUE TO BE ACTUAL RADIUS
  }

  public static class ArmConstants {
    public static final int kArmMotorPort = 0;
  }
}
