package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakePivotMotor;
  private CANSparkMax intakeMotor;
  private RelativeEncoder intakePivotEncoder;
  private SparkPIDController intakePID;

  private DigitalInput intakeEndstop;
  private DigitalInput hasGamePieceDigitalInput;

  private IntakeState state = IntakeState.AT_POSITION;
  private double intakeTarget;
  public boolean hasGamePeice = false;

  public Intake() {
    intakePivotMotor =
      new CANSparkMax(
        IntakeConstants.kIntakePivotMotorID,
        MotorType.kBrushless
      );
    intakeMotor =
      new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

    intakePivotEncoder =
      intakePivotMotor.getAlternateEncoder(Constants.kThroughBoreEncoderRev);

    intakePID = intakePivotMotor.getPIDController();
    intakePID.setFeedbackDevice(intakePivotEncoder);

    intakePID.setP(IntakeConstants.kIntakeP);
    intakePID.setI(IntakeConstants.kIntakeI);
    intakePID.setD(IntakeConstants.kIntakeD);

    intakePID.setSmartMotionAllowedClosedLoopError(IntakeConstants.kIntakeAngleTolerance, 0);

    intakeEndstop = new DigitalInput(IntakeConstants.kHomePositionLimitDIO);
    hasGamePieceDigitalInput = new DigitalInput(IntakeConstants.kHasGamePieceLimitDIO);
  }

  public void cleanIntake(){
    intakeMotor.set(0.05);
  }


  public void updateHasGamePiece(){
  hasGamePeice = hasGamePieceDigitalInput.get();
  }

  public void zeroIntake(){
      intakePivotEncoder.setPosition(0);
  }

  public void endStopProtection(){
    if (intakeEndstop.get()){
      zeroIntake();
      intakePivotMotor.stopMotor();
      intakeTarget = 0;
    }
  }

  public void home(){
    if (intakePivotEncoder.getPosition() > 0){
      intakeMotor.set(-0.05);
    }
  }

  public IntakeState getState() {
    return state;
  }

  public double getIntakeAngle() {
    double angleInDegrees = (intakePivotEncoder.getPosition() * 360) / IntakeConstants.kIntakeAngleRatio;
    return angleInDegrees;
}


  public void setIntakeAngle(double target){
    intakeTarget = target;
  }

  public void intake(){
    intakeMotor.set(1);
  }

  public void outtake(){
    intakeMotor.set(-1);
  }

  public void stop(){
    intakeMotor.set(0);
  }

  public void updateIntakeState(){
    if (Math.abs(getIntakeAngle() - intakeTarget) > IntakeConstants.kIntakeAngleTolerance){
      state = IntakeState.MOVING;
    } else {
      state = IntakeState.AT_POSITION;
    }
  }
  
  @Override
  public void periodic() {
    endStopProtection();
    updateHasGamePiece();
    intakePID.setReference(intakeTarget, CANSparkMax.ControlType.kPosition);
    updateIntakeState();
  }

  public enum IntakeState {
    MOVING,
    AT_POSITION,
  }
}
