package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakePivotMotor;
  private CANSparkMax intakeMotor;
  private SparkAbsoluteEncoder intakePivotEncoder;
  private SparkPIDController intakePID;

  private RelativeEncoder limitEncoder;

  private DigitalInput intakeEndstop;
  private DigitalInput hasGamePieceDigitalInput;

  private IntakeState state = IntakeState.AT_POSITION;
  private double intakeTarget;
  public boolean hasGamePiece;
  private double intakeTargetAngle;

  public Intake() {
    intakePivotMotor =
      new CANSparkMax(
        IntakeConstants.kIntakePivotMotorID,
        MotorType.kBrushless
      );
    intakeMotor =
      new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

    limitEncoder = intakePivotMotor.getEncoder();
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakePivotEncoder = intakePivotMotor.getAbsoluteEncoder();

    intakePID = intakePivotMotor.getPIDController();
    intakePID.setFeedbackDevice(intakePivotEncoder);

    intakePID.setP(IntakeConstants.kIntakeP);
    intakePID.setI(IntakeConstants.kIntakeI);
    intakePID.setD(IntakeConstants.kIntakeD);
    intakePID.setSmartMotionMaxAccel(IntakeConstants.kIntakeAcceleration, 0);

    intakePID.setSmartMotionAllowedClosedLoopError(
      IntakeConstants.kIntakeAngleTolerance,
      0
    );

    intakePID.setPositionPIDWrappingEnabled(false);
    intakePID.setPositionPIDWrappingMinInput(0);
    intakePID.setPositionPIDWrappingMaxInput(1);
    intakePID.setOutputRange(-1, 1);

    intakeMotor.burnFlash();

    hasGamePieceDigitalInput =
      new DigitalInput(IntakeConstants.kHasGamePieceLimitDIO);
  }

  public void cleanIntake() {
    intakeMotor.set(0.05);
  }

  public void updateHasGamePiece() {
    if (!hasGamePieceDigitalInput.get()) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }
  }

  public void home() {
    if (intakePivotEncoder.getPosition() > 0) {
      intakeMotor.set(-0.05);
    }
  }

  public IntakeState getState() {
    return state;
  }

  public double getIntakeAngle() {
    double angleInDegrees = (intakePivotEncoder.getPosition() * 360);
    return angleInDegrees;
  }

  public void setIntakeAngle(double target) {
    intakeTarget = target / 360;
    intakeTarget += IntakeConstants.kIntakeOffset;
    intakeTargetAngle = intakeTarget * 360;
    intakePID.setReference(intakeTarget, CANSparkMax.ControlType.kPosition);
  }

  public void intake() {
    intakeMotor.set(1);
  }

  public void outtake() {
    intakeMotor.set(-1);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void updateIntakeState() {
    if (
      Math.abs(getIntakeAngle() - intakeTarget) >
      IntakeConstants.kIntakeAngleTolerance
    ) {
      state = IntakeState.MOVING;
    } else {
      state = IntakeState.AT_POSITION;
    }
  }

  public void endStopProtection() {
    if (!intakeEndstop.get()) {
      limitEncoder.setPosition(0);
    }
  }

  public void debugValues() {
    SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
    SmartDashboard.putNumber(
      "Intake Encoder Val",
      intakePivotEncoder.getPosition()
    );
    SmartDashboard.putNumber("Intake Target", intakeTarget);
    SmartDashboard.putNumber("Intake Target Angle", intakeTargetAngle);
    SmartDashboard.putBoolean("Piece", !hasGamePieceDigitalInput.get());
    SmartDashboard.putNumber(
      "Relative Intake Position",
      limitEncoder.getPosition()
    );
  }

  @Override
  public void periodic() {
    //endStopProtection();
    updateHasGamePiece();
    // intakePID.setReference(intakeTarget, CANSparkMax.ControlType.kPosition);
    updateIntakeState();
    debugValues();
  }

  public enum IntakeState {
    MOVING,
    AT_POSITION,
  }
}
