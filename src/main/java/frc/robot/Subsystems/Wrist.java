package frc.robot.Subsystems;

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

  private WristState state = WristState.AT_POSITION;
  private double wristTarget;

  public Wrist() {
    wristMotor =
      new CANSparkMax(WristConstants.kWristMotorID, MotorType.kBrushless);

    wristEncoder =
     wristMotor.getAlternateEncoder(Constants.kThroughBoreEncoderRev);

    wristPID = wristMotor.getPIDController();
    wristPID.setFeedbackDevice(wristEncoder);

    wristPID.setP(WristConstants.kWristP);
    wristPID.setI(WristConstants.kWristI);
    wristPID.setD(WristConstants.kWristD);
    wristPID.setSmartMotionAllowedClosedLoopError(WristConstants.kwristAngleTolerance, 0);
  }

  public void setWristAngle(double target) {
    wristTarget = target;
  }

public double getWristAngle() {
    double angleInDegrees = (wristEncoder.getPosition() * 360);
    
    return angleInDegrees;
}


  public WristState getState(){
    return state;
  }

  private void updateWristState(){
    if (Math.abs(getWristAngle() - wristTarget) > WristConstants.kwristAngleTolerance){
      state = WristState.MOVING;
    } else {
      state = WristState.AT_POSITION;
    }
  }

  @Override
  public void periodic() {
    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kPosition);
    updateWristState();
  }

  public enum WristState {
    MOVING,
    AT_POSITION,
  }
}
