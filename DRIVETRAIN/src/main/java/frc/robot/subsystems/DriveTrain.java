// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonFX left1 = new WPI_TalonFX(DriveTrainConstants.kLeftTalonID1);
  private final WPI_TalonFX right1 = new WPI_TalonFX(DriveTrainConstants.kRightTalonID1);

  private final MotorControllerGroup leftMotor = new MotorControllerGroup(left1, new WPI_TalonFX(DriveTrainConstants.kLeftTalonID2));
  private final MotorControllerGroup rightMotor = new MotorControllerGroup(right1, new WPI_TalonFX(DriveTrainConstants.kRightTalonID2));

  public final TalonFXSensorCollection sensorRight =  new TalonFXSensorCollection(right1);
  public final TalonFXSensorCollection sensorLeft =  new TalonFXSensorCollection(left1);
  
  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  private final AHRS gyroscope = new AHRS();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyroscope.getRotation2d(), sensorRight.getIntegratedSensorPosition() * DriveTrainConstants.sensorFactor, sensorLeft.getIntegratedSensorPosition() * DriveTrainConstants.sensorFactor);
  private final Field2d field = new Field2d();
   
  public DriveTrain() {
    leftMotor.setInverted(true);
    SmartDashboard.putData("field", field);
  }


  public void setMotors(double left, double right) {
    leftMotor.set(left);
    rightMotor.set(right);
  }


  public DifferentialDrive getDriveTrain() {
    return drive;
  }
  @Override
  public void periodic() {

    odometry.update(gyroscope.getRotation2d(), sensorRight.getIntegratedSensorPosition() * DriveTrainConstants.sensorFactor, sensorLeft.getIntegratedSensorPosition() * DriveTrainConstants.sensorFactor);
    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {

  }
}