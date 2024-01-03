// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  
  // store swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // store a playstation controller
  private final PS4Controller driverJoystick = new PS4Controller(OIConstants.kControllerPort);

  public RobotContainer() {

    // by default the swerve will run the joystick command
    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoystick.getRightY(),
        () -> driverJoystick.getRightX(),
        () -> driverJoystick.getLeftX(),
        () -> !driverJoystick.getL1Button()
      ));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {

    // this where you would return command compositions with trajectories and angle to rotate
    //    return new SwerveTrajectoryCommand("filepath/filename.json", swerveSubsystem.getHeading(), swerveSubsystem);
    
    return Commands.print("No autonomous command configured");

  }
}
