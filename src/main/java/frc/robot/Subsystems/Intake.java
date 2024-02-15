package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakePivotMotor;
  private CANSparkMax intakeMotor;
  private RelativeEncoder intakePivotEncoder;
  private SparkPIDController intakePID;

  private IntakeState state = IntakeState.AT_POSITION;
  private double intakeTarget;

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
  }

  public IntakeState getIntakeState() {
    return state;
  }

  public double getIntakeaAngle(){
    return intakePivotEncoder.getPosition();
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
    if (Math.abs(getIntakeaAngle() - intakeTarget) > IntakeConstants.kIntakeAngleTolerance){
      state = IntakeState.MOVING;
    } else {
      state = IntakeState.AT_POSITION;
    }
  }
  
  @Override
  public void periodic() {
    intakePID.setReference(intakeTarget, CANSparkMax.ControlType.kPosition);
    updateIntakeState();
  }

  public enum IntakeState {
    MOVING,
    AT_POSITION,
  }
}
