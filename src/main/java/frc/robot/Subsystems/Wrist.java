package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.WristConstants;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristMotor;
  private RelativeEncoder wristEncoder;
  private SparkPIDController wristPID;

  private WristState state = WristState.IDLE;
  private double wristPosition;
  private double wristTarget;

  public Wrist() {
    wristMotor =
      new CANSparkMax(WristConstants.kWristMotorID, MotorType.kBrushless);

    wristEncoder =
      wristMotor.getAlternateEncoder(Constants.kThroughBoreEncoderRev);

    wristPID = wristMotor.getPIDController();
    wristPID.setFeedbackDevice(wristEncoder);
  }

  public WristState getWristState() {
    return state;
  }

  public void stopWrist() {
    wristMotor.set(0);
    state = WristState.STOPPED;
  }

  public double getWristAngle() {
    wristPosition = wristEncoder.getPosition();
    return wristPosition;
  }

  public void setWristAngle(double angle) {
    wristTarget = angle;
    state = WristState.MOVING;
    wristPID.setReference(angle, ControlType.kPosition);
    // mess with the pid values
  }

  public void updateWristState() {
    switch (state) {
      case IDLE:
        break;
      case MOVING:
        if (
          Math.abs(getWristAngle() - wristTarget) <
          WristConstants.kwristAngleTolerance
        ) {
          state = WristState.AT_POSITION;
        }
        break;
      case AT_POSITION:
        // hold position. this should be done with pid too ig
        break;
      case STOPPED:
        stopWrist();
        state = WristState.IDLE;
        break;
    }
  }

  public enum WristState {
    IDLE,
    MOVING,
    AT_POSITION,
    STOPPED,
  }
}
