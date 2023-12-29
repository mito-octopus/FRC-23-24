package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveTrainConstants;

public class SwerveModule {

    // motors (also contains encoder input built-in)
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX turnMotor;

    // pid for angle and speed
    private final PIDController turningPIDController;
    private final PIDController speedPIDController;

    // offset if needed
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;


    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, boolean absoluteEncoderReversed, double absoluteEncoderOffset){
        // offsets if needed
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        // initialize motors with provided ids
        this.driveMotor = new WPI_TalonFX(driveMotorId);
        this.turnMotor = new WPI_TalonSRX(turnMotorId);

        // set everything to default values
        this.driveMotor.configFactoryDefault();
        this.turnMotor.configFactoryDefault();

        // set inverted if needed
        this.driveMotor.setInverted(driveMotorReversed);
        this.turnMotor.setInverted(turnMotorReversed);

        // turn motor uses an absolute mag encoder
        this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        // drive motor uses integrated sensor
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        // pid controller
        turningPIDController = new PIDController(DriveTrainConstants.kPTurning, DriveTrainConstants.kITurning, DriveTrainConstants.kDTurning);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        speedPIDController = new PIDController(DriveTrainConstants.kPDrive, DriveTrainConstants.kIDrive, DriveTrainConstants.kDDrive);

        // reset everything
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * DriveTrainConstants.driveEncoderToMeters;
    }

    public double getTurningPosition() {
        return turnMotor.getSelectedSensorPosition() * DriveTrainConstants.turnEncoderToRad;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * DriveTrainConstants.driveEncoderToMeters;
    }

    public double getTurnVelocity() {
        return turnMotor.getSelectedSensorVelocity() * DriveTrainConstants.turnEncoderToRad;
    }

    public double getAbsoluteTurnEncoder() {
        this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        double angle = turnMotor.getSelectedSensorPosition() * DriveTrainConstants.turnEncoderToRad;
        this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        angle -= absoluteEncoderOffset;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(getAbsoluteTurnEncoder());
        return;
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }


    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(speedPIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
        turnMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

}
