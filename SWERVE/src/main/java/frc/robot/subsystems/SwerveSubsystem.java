// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class SwerveSubsystem extends SubsystemBase {

  // store the 4 modules
  private final SwerveModule topLeft = new SwerveModule(
    DriveTrainConstants.topLeftDriveMotorID,
    DriveTrainConstants.topLeftTurnMotorID,
    DriveTrainConstants.topLeftDriveMotorReversed,
    DriveTrainConstants.topLeftTurnMotorReversed,
    DriveTrainConstants.topLeftAbsoluteEncoderReversed,
    DriveTrainConstants.topLeftAbsoluteEncoderOffset
  );

  private final SwerveModule topRight = new SwerveModule(
    DriveTrainConstants.topRightDriveMotorID,
    DriveTrainConstants.topRightTurnMotorID,
    DriveTrainConstants.topRightDriveMotorReversed,
    DriveTrainConstants.topRightTurnMotorReversed,
    DriveTrainConstants.topRightAbsoluteEncoderReversed,
    DriveTrainConstants.topRightAbsoluteEncoderOffset
  );

  private final SwerveModule bottomLeft = new SwerveModule(
    DriveTrainConstants.bottomLeftDriveMotorID,
    DriveTrainConstants.bottomLeftTurnMotorID,
    DriveTrainConstants.bottomLeftDriveMotorReversed,
    DriveTrainConstants.bottomLeftTurnMotorReversed,
    DriveTrainConstants.bottomLeftAbsoluteEncoderReversed,
    DriveTrainConstants.bottomLeftAbsoluteEncoderOffset
  );

  private final SwerveModule bottomRight = new SwerveModule(
    DriveTrainConstants.bottomRightDriveMotorID,
    DriveTrainConstants.bottomRightTurnMotorID,
    DriveTrainConstants.bottomRightDriveMotorReversed,
    DriveTrainConstants.bottomRightTurnMotorReversed,
    DriveTrainConstants.bottomRightAbsoluteEncoderReversed,
    DriveTrainConstants.bottomRightAbsoluteEncoderOffset
  );

  private final AHRS gyroscope = new AHRS();

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    DriveTrainConstants.kDriveKinematics, 
    gyroscope.getRotation2d(),
    new SwerveModulePosition[] {
      topLeft.getPosition(),
      topRight.getPosition(),
      bottomLeft.getPosition(),
      bottomRight.getPosition()
    }
  );

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){}
    }).start();
  }

  public void zeroHeading() {
    gyroscope.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyroscope.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry (Pose2d pose) {
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      topLeft.getPosition(), topRight.getPosition(),
      bottomLeft.getPosition(), bottomRight.getPosition()
    }, pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      topLeft.getPosition(), topRight.getPosition(),
      bottomLeft.getPosition(), bottomRight.getPosition()
    });

    
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
    topLeft.stop();
    topRight.stop();
    bottomLeft.stop();
    bottomRight.stop();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = DriveTrainConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveTrainConstants.physicalMaxSpeed);
    topLeft.setDesiredState(desiredStates[0]);
    topRight.setDesiredState(desiredStates[1]);
    bottomLeft.setDesiredState(desiredStates[2]);
    bottomRight.setDesiredState(desiredStates[3]);
  }
}
