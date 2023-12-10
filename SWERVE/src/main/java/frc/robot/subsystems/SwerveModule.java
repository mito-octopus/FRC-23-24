package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;

public class SwerveModule {

    // motors (also contains encoder input built-in)
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX turnMotor;

    private final PIDController turningPidController;

    private final boolean encoderReversed;
    private final double encoderOffset;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int encoderOffset, boolean encoderReversed){
        this.driveMotor = new WPI_TalonFX(driveMotorId);
        this.turnMotor = new WPI_TalonSRX(turnMotorId);

        this.driveMotor.setInverted(driveMotorReversed);
        this.turnMotor.setInverted(turnMotorReversed);

        this.encoderOffset = encoderOffset;
        this.encoderReversed = encoderReversed;

    }


}
