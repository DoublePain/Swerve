package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final TalonFX m_driveMotor;
    private final TalonSRX m_turningMotor;

    private final CANcoder TurnEncoder;
    private final CANcoder DriveEncoder;
             public int CANcoderChannel;
    private final PIDController turningPidController;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int CANcoderChannel) {
        m_driveMotor = new TalonFX(driveMotorId);
        m_turningMotor = new TalonSRX(turningMotorId);

        m_driveMotor.setInverted(driveMotorReversed);
        m_turningMotor.setInverted(turningMotorReversed);
     
        TurnEncoder = new CANcoder(CANcoderChannel);
        DriveEncoder = new CANcoder(CANcoderChannel);

        DriveEncoder.setPosition(ModuleConstants.kDriveEncoderRot2Meter);
        DriveEncoder.setPosition(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        TurnEncoder.setPosition(ModuleConstants.kTurningEncoderRot2Rad);
        TurnEncoder.setPosition(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDrivePosition() {
        return DriveEncoder.getPosition().getValue();
    }

    public double getTurningPosition() {
        return TurnEncoder.getPosition().getValue();
    }

    public double getDriveVelocity() {
        return DriveEncoder.getVelocity().getValue();
    }

    public double getTurningVelocity() {
        return TurnEncoder.getVelocity().getValue();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        m_driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        m_turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turningMotor.set(ControlMode.PercentOutput, 0);
    }
}