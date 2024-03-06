package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final CANSparkMax intakePivotMotor;
  private final CANSparkMax intakeMotor;
  private final SparkAbsoluteEncoder intakePivotEncoder;
  private final SparkPIDController intakePID;
    private final DigitalInput hasGamePieceDigitalInput;
  private IntakeState state = IntakeState.AT_POSITION;
  private double intakeTarget;
  public boolean hasGamePiece;
  private double intakeTargetAngle;

    public Timer intakeTimer = new Timer();


  public Intake() {
    intakePivotMotor =
      new CANSparkMax(
        IntakeConstants.kIntakePivotMotorID,
        MotorType.kBrushless
      );
    intakeMotor =
      new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

    intakeMotor.setSmartCurrentLimit(40);

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


    intakeMotor.setIdleMode(IdleMode.kCoast);
    debugValues();
  }

  public void cleanIntake() {
    intakeMotor.set(0.05);
  }

  public void updateHasGamePiece() {
      hasGamePiece = !hasGamePieceDigitalInput.get();
  }
  public IntakeState getState() {
    return state;
  }

  public double getIntakeAngle() {
      return (intakePivotEncoder.getPosition() * 360);
  }

  public void setIntakeAngle(double target) {
    intakeTarget = target / 360;
    intakeTarget += IntakeConstants.kIntakeOffset;
    intakeTargetAngle = intakeTarget * 360;
    intakePID.setReference(intakeTarget, CANSparkMax.ControlType.kPosition);
    state = IntakeState.MOVING;
  }

  public void intake() {
    intakeMotor.set(1);
  }

  public void outtake() {
    intakeMotor.set(-0.5);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void toggleIntakeMotorIdleMode(){
    if(intakePivotMotor.getIdleMode() == IdleMode.kBrake){
      intakePivotMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      intakePivotMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  public void updateIntakeState() {
    if (
      Math.abs(intakePivotEncoder.getPosition() - intakeTarget) >
      IntakeConstants.kIntakeAngleTolerance
    ) {
      state = IntakeState.MOVING;
    } else {
      state = IntakeState.AT_POSITION;
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
    SmartDashboard.putBoolean("Intake Piece", !hasGamePieceDigitalInput.get());
    SmartDashboard.putString("intake State", state.toString());
    SmartDashboard.putString("Intake Idle Mode", intakePivotMotor.getIdleMode().toString());
    SmartDashboard.putNumber("IntakeTimer", intakeTimer.get());
    SmartDashboard.putNumber("INTAKE ANGLE", getIntakeAngle());

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
