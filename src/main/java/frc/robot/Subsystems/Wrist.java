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

  private WristState state = WristState.AT_POSITION;
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

  public void periodic() {
    //updateWristState();

    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kPosition);
  }


  private void movingState(){
  }

  private void atPositionState(){
    if (Math.abs(wristTarget - wristPosition) > WristConstants.kwristAngleTolerance) {
      wristPID.setReference(wristPosition, CANSparkMax.ControlType.kPosition);      
    } else {
      wristPID.setReference(wristPosition, CANSparkMax.ControlType.kPosition);
    
    }
  }

  public void updateWristState() {
    switch (state) {
      case MOVING:
      movingState();
        break;
      case AT_POSITION:
      atPositionState();
        break;
    }
  }

  public enum WristState {
    MOVING,
    AT_POSITION,
  }
}
