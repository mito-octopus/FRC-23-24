// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

  // store the subsystem being used
  private final SwerveSubsystem swerveSubsystem;

  // functions that will get joystick values
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  // limit acceleration of the bot
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {

    // store functions and subsystem
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;

    // create new rate limiters with the desired max accelerations 
    this.xLimiter = new SlewRateLimiter(DriveTrainConstants.kDriveMaxAcceleration);
    this.yLimiter = new SlewRateLimiter(DriveTrainConstants.kDriveMaxAcceleration);
    this.turningLimiter = new SlewRateLimiter(DriveTrainConstants.kDriveMaxAngularAccelerationDegrees);

    // require the swerve subsystem
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get values
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    // apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;

    // rate limit
    xSpeed = xLimiter.calculate(xSpeed) * DriveTrainConstants.kDriveMaxVelocity/4;
    ySpeed = yLimiter.calculate(ySpeed)* DriveTrainConstants.kDriveMaxVelocity/4;
    turningSpeed = turningLimiter.calculate(turningSpeed)* DriveTrainConstants.kDriveMaxAngularAccelerationDegrees;
  
    // convert to chassis speeds, optionally making them field relative
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
    
    // apply the speeds
    swerveSubsystem.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the subsystem
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // don't end unless interrupted
    return false;
  }
}
