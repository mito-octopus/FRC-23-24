// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTrajectoryCommand extends CommandBase {
  
  List<Trajectory.State> trajectoryStates;
  SwerveSubsystem swerveSubsystem;
  HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(DriveTrainConstants.kPx, DriveTrainConstants.kIx, DriveTrainConstants.kDx), 
    new PIDController(DriveTrainConstants.kPy, DriveTrainConstants.kIy, DriveTrainConstants.kDy), 
    new ProfiledPIDController(DriveTrainConstants.kPr, DriveTrainConstants.kIr, DriveTrainConstants.kDr,
      new TrapezoidProfile.Constraints(DriveTrainConstants.kMaxRotVelocityRad, DriveTrainConstants.kMaxRotAccelerationRad)
    ));
  int currentState;
  Rotation2d rotation;

  /** Creates a new SwerveTrajectoryCommand. */
  public SwerveTrajectoryCommand(String filename, double RotationAngle, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    Trajectory trajectory = new Trajectory();

		// try to load the file trajectory into the trajectory object
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
    }
    
    this.trajectoryStates = trajectory.getStates();
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    currentState = 0;
    rotation = new Rotation2d(RotationAngle * Math.PI / 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = trajectoryStates.get(currentState);
    ChassisSpeeds adjustedSpeeds = controller.calculate(swerveSubsystem.getPose(), goal, rotation);
    swerveSubsystem.setChassisSpeeds(adjustedSpeeds);
    currentState++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentState > trajectoryStates.size()){
      return true;
    }
    return false;
  }
}
