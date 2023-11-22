// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonFX left1 = new WPI_TalonFX(0);
  private final WPI_TalonFX right1 = new WPI_TalonFX(1);
  private final WPI_TalonFX left2 = new WPI_TalonFX(2);
  private final WPI_TalonFX right2 = new WPI_TalonFX(3);

  private final MotorControllerGroup leftMotor = new MotorControllerGroup(left1, left2);
  private final MotorControllerGroup rightMotor = new MotorControllerGroup(right1, right2);

  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  private final Encoder leftEncoder = new Encoder(DriveTrainConstants.kEncoderPortLeft1, DriveTrainConstants.kEncoderPortLeft2);  /** Creates a new drivetrain. */
  private final Encoder rightEncoder = new Encoder(DriveTrainConstants.kEncoderPortRight1, DriveTrainConstants.kEncoderPortRight2);  /** Creates a new drivetrain. */
  private final AHRS gyroscope = new AHRS();

  private final DifferentialDriveOdometry odometry;

  private final Field2d field = new Field2d();
  
  public DriveTrain() {
    leftEncoder.setDistancePerPulse(2 * Math.PI * 10); 
    rightEncoder.setDistancePerPulse(2 * Math.PI * 10); 
    odometry = new DifferentialDriveOdometry(gyroscope.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  
    SmartDashboard.putData("Field", field);
  }


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
    Pose2d pose = odometry.update(gyroscope.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  
    field.setRobotPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
  }
}
