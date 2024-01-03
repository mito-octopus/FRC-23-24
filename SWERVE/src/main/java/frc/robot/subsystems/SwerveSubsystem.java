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
  private final SwerveModule frontLeft = new SwerveModule(
    DriveTrainConstants.kFrontLeftDriveMotorID,
    DriveTrainConstants.kFrontLeftTurnMotorID,
    DriveTrainConstants.kFrontLeftDriveMotorReversed,
    DriveTrainConstants.kFrontLeftTurnMotorReversed,
    DriveTrainConstants.kFrontLeftAbsoluteEncoderReversed,
    DriveTrainConstants.kFrontLeftAbsoluteEncoderOffset
  );

  private final SwerveModule frontRight = new SwerveModule(
    DriveTrainConstants.kFrontRightDriveMotorID,
    DriveTrainConstants.kFrontRightTurnMotorID,
    DriveTrainConstants.kFrontRightDriveMotorReversed,
    DriveTrainConstants.kFrontRightTurnMotorReversed,
    DriveTrainConstants.kFrontRightAbsoluteEncoderReversed,
    DriveTrainConstants.kFrontRightAbsoluteEncoderOffset
  );

  private final SwerveModule backLeft = new SwerveModule(
    DriveTrainConstants.kBackLeftDriveMotorID,
    DriveTrainConstants.kBackLeftTurnMotorID,
    DriveTrainConstants.kBackLeftDriveMotorReversed,
    DriveTrainConstants.kBackLeftTurnMotorReversed,
    DriveTrainConstants.kBackLeftAbsoluteEncoderReversed,
    DriveTrainConstants.kBackLeftAbsoluteEncoderOffset
  );

  private final SwerveModule backRight = new SwerveModule(
    DriveTrainConstants.kBackRightDriveMotorID,
    DriveTrainConstants.kBackRightTurnMotorID,
    DriveTrainConstants.kBackRightDriveMotorReversed,
    DriveTrainConstants.kBackRightTurnMotorReversed,
    DriveTrainConstants.kBackRightAbsoluteEncoderReversed,
    DriveTrainConstants.kBackRightAbsoluteEncoderOffset
  );

  // store gyroscope 

  private final AHRS gyroscope = new AHRS();

  // store odometry

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    DriveTrainConstants.kDriveKinematics, 
    gyroscope.getRotation2d(),
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    }
  );

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    // zero the heading (this has to be done assymetrically idrk why)
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){}
    }).start();
  }

  // getters

  public double getHeading() {
    return Math.IEEEremainder(gyroscope.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  // setters

  public void setChassisSpeeds(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = DriveTrainConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveTrainConstants.kDriveMaxVelocity);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  // resetters 

  public void zeroHeading() {
    gyroscope.reset();
  }

  public void resetOdometry (Pose2d pose) {
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backLeft.getPosition(), backRight.getPosition()
    }, pose);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }


  // periodic function

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backLeft.getPosition(), backRight.getPosition()
    });

    
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

}
