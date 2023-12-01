// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  private final DriveTrain driveTrain;
  private final Supplier<Double> speedFunction, turnFunction;
  private final SlewRateLimiter speedLimiter, turnLimiter;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrain driveTrain, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;

    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

    this.speedLimiter = new SlewRateLimiter(DriveTrainConstants.kMaxAccelerationUnitsPerSecond);
    this.turnLimiter = new SlewRateLimiter(DriveTrainConstants.kMaxAccelerationUnitsPerSecond);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.getDriveTrain().arcadeDrive(speedLimiter.calculate(speedFunction.get()) * 5/4, turnLimiter.calculate(turnFunction.get()) * 5/4);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
