package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.WristConstants;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristMotor;
  private SparkAbsoluteEncoder wristEncoder;
  private SparkPIDController wristPID;

  private WristState state = WristState.AT_POSITION;
  private double wristTarget = 0;
  private double wristTargetAngle = 0;

  private DigitalInput wristEndstop;

  public Wrist() {
    wristMotor =
      new CANSparkMax(WristConstants.kWristMotorID, MotorType.kBrushless);

    wristEncoder =
     wristMotor.getAbsoluteEncoder();

    wristPID = wristMotor.getPIDController();
    wristPID.setFeedbackDevice(wristEncoder);

    wristPID.setP(WristConstants.kWristP);
    wristPID.setI(WristConstants.kWristI);
    wristPID.setD(WristConstants.kWristD);
    wristPID.setFF(WristConstants.kWristFF);
    wristPID.setSmartMotionAllowedClosedLoopError(WristConstants.kwristAngleTolerance, 0);

    wristPID.setPositionPIDWrappingEnabled(true);
    wristPID.setPositionPIDWrappingMinInput(WristConstants.kEncoderPositionPIDMinInput);
    wristPID.setPositionPIDWrappingMaxInput(WristConstants.kEncoderPositionPIDMaxInput);
    wristPID.setOutputRange(-1,1);

    wristEndstop = new DigitalInput(WristConstants.kWristEndstopDIO);

    wristMotor.burnFlash();
  }

  public void setWristAngle(double target) {
    wristTargetAngle = target;
    wristTarget = target / 360;
  }

public double getWristAngle() {
  return wristEncoder.getPosition() * 360;
}

public double getWristEncoderVal(){
  return wristEncoder.getPosition();
}

// public void zeroWrist(){
//   wristEncoder.setZeroOffset(-wristEncoder.getPosition());
// }

// public void endStopProtection(){
//   if (wristEndstop.get()){
//     zeroWrist();
//     wristMotor.stopMotor();
//     wristTarget = 0;
//   }
// }

public void home(){
  if (wristEncoder.getPosition() > 0){
    wristMotor.set(-0.05);
  }
}

  public WristState getState(){
    return state;
  }

  private void updateWristState(){
    if (Math.abs(getWristEncoderVal() - wristTarget) > WristConstants.kwristAngleTolerance){
      state = WristState.MOVING;
    } else {
      state = WristState.AT_POSITION;
    }
  }

  public void debugValues(){
    SmartDashboard.putNumber("Encoder Val", getWristEncoderVal());
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putNumber("Wrist Target", wristTarget);
    SmartDashboard.putNumber("Wrist Target Angle", wristTargetAngle);
    SmartDashboard.putString("Wrist State", getState().toString());
    SmartDashboard.putNumber("Wrist Encoder Offset", wristEncoder.getZeroOffset());
    SmartDashboard.putBoolean("Wrist Home Endstop", wristEndstop.get());
  }

  public void competitionValues(){
  }

  @Override
  public void periodic() {
    //endStopProtection();
    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kPosition);
    updateWristState();
    // debugValues();
    //competitionValues();
  }

  public enum WristState {
    MOVING,
    AT_POSITION,
  }
}
