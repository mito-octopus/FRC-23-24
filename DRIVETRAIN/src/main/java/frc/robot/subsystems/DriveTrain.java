// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private final MotorControllerGroup leftMotor = new MotorControllerGroup(new TalonFX(), new TalonFX());
  private final MotorControllerGroup rightMotor = new MotorControllerGroup(new TalonFX(), new TalonFX());
  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  private final Encoder leftEncoder = new Encoder(DriveTrainConstants.kEncoderPortLeft1, DriveTrainConstants.kEncoderPortLeft2);  /** Creates a new drivetrain. */
  private final Encoder rightEncoder = new Encoder(DriveTrainConstants.kEncoderPortRight1, DriveTrainConstants.kEncoderPortRight2);  /** Creates a new drivetrain. */
  private final AnalogGyro gyroscope = new  AnalogGyro(0);

  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(gyroscope);

  // These are our EncoderSim objects, which we will only use in
// simulation. However, you do not need to comment out these
// declarations when you are deploying code to the roboRIO.
  private EncoderSim m_leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(rightEncoder);

  private final DifferentialDriveOdometry odometry;

  private final Field2d field = new Field2d();

    // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
    7.29,                    // 7.29:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    60.0,                    // The mass of the robot is 60 kg.
    Units.inchesToMeters(3), // The robot uses 3" radius wheels.
    0.7112,                  // The track width is 0.7112 meters.

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  
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
    m_driveSim.setInputs(leftMotor.get() * RobotController.getInputVoltage(),
                         rightMotor.get() * RobotController.getInputVoltage());
  
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);
  
    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }
}
