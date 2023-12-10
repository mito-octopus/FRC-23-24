// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RaiseToAngle extends CommandBase {
  PIDController pid = new PIDController(2, 0, 0);
  Arm arm;
  double angle;
  /** Creates a new RaiseToAngle. */
  public RaiseToAngle(Arm arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.arm = arm;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setMotor(pid.calculate(arm.getArmAngleDegrees(), angle))
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
