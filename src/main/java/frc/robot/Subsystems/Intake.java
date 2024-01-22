package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
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

  private IntakeState state = IntakeState.IDLE;
  private double pivotPosition;
  private double pivotTarget;

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

  public void stopIntakePivot() {
    intakePivotMotor.set(0);
    state = IntakeState.STOPPED;
  }

  public double getIntakeAngle() {
    pivotPosition = intakePivotEncoder.getPosition();
    return pivotPosition;
  }

  public void runIntakeWheels() {
    intakeMotor.set(1);
  }

  public void stopIntakeWheels() {
    intakeMotor.set(0);
  }

  public void reverseIntakeWheels() {
    intakeMotor.set(-1);
  }

  public void setIntakeAngle(double angle) {
    pivotTarget = angle;
    state = IntakeState.MOVING;
    intakePID.setReference(angle, ControlType.kPosition);
    // pid values and stuff
  }

  public void updateIntakeState() {
    switch (state) {
      case IDLE:
        break;
      case MOVING:
        if (
          Math.abs(getIntakeAngle() - pivotTarget) <
          IntakeConstants.kIntakeAngleTolerance
        ) {
          state = IntakeState.AT_POSITION;
        }
        break;
      case AT_POSITION:
        //hold the position somehow
        break;
      case STOPPED:
        stopIntakePivot();
        break;
    }
  }

  public void periodic() {
    updateIntakeState();
  }

  public enum IntakeState {
    IDLE,
    MOVING,
    AT_POSITION,
    STOPPED,
  }
}
