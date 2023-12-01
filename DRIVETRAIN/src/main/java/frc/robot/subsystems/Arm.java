// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final Spark armMotor = new Spark(ArmConstants.kArmMotorPort);
  private final AnalogPotentiometer armPotentiometer = new AnalogPotentiometer(ArmConstants.kPotPort, 180, 30);
  /** Creates a new Arm. */
  public Arm() {
  }

  public void setMotor(double speed){
    armMotor.set(speed);
  }

  public double getArmAngleDegrees(){
    return armPotentiometer.get() * 180;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
