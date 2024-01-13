package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.ModuleSpecificConstants;

public class SwerveModule {


    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turningpiController;

    private final CANcoder absoluteEnCoder;
    private final double absoluteEnCoderOffset;
    private final boolean absoluteEnCoderReversed;

    private final ModuleSpecificConstants constants;


    public SwerveModule(int moduleID, String moduleLocation)
    {
        constants = new ModuleSpecificConstants(moduleID, moduleLocation);

        absoluteEnCoder = new CANcoder(constants.kAbsoluteEncoderID);
        absoluteEnCoderOffset = constants.kAbsoluteEncoderOffset;
        absoluteEnCoderReversed = constants.kAbsoluteEncoderReversed;

        driveMotor = new CANSparkMax(constants.kdriveMotorID, CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(constants.kturningMotorID, CANSparkMax.MotorType.kBrushless);

        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveMotor.setInverted(constants.kdriveEncoderReversed);
        turnMotor.setInverted(constants.kturningEncoderReversed);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningpiController = new PIDController(ModuleConstants.kPTurning, 0, 0);

        turningpiController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }
    
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    
    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsolutePositionRadians() {
        return Math.toRadians(getAbsolutePositionDegrees());
    }

    public double getAbsolutePositionDegrees() {
        double rawPosition = absoluteEnCoder.getAbsolutePosition().getValue() + absoluteEnCoderOffset;
        double angle = rawPosition * 360.0;
        return (angle * (absoluteEnCoderReversed ? -1 : 1));
    }
    
    

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsolutePositionRadians());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnPosition()));
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < 0.1) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / constants.kMaxModuleSpeed);
        turnMotor.set(turningpiController.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public double getSpeed() {
        return constants.kMaxModuleSpeed;
    }
}
