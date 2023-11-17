// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private final Spark leftMotor = new Spark(DriveTrainConstants.kLeftMotorPort);
  private final Spark rightMotor = new Spark(DriveTrainConstants.kRightMotorPort);
  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  /** Creates a new drivetrain. */
  public DriveTrain() {}
  public void setMotors(double left, double right) {
    leftMotor.setVoltage(left);
    rightMotor.setVoltage(right);
  }
  public DifferentialDrive getDriveTrain() {
    return drive;
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
