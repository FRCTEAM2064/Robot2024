package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
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

  public Wrist() {
    wristMotor =
      new CANSparkMax(WristConstants.kWristMotorID, MotorType.kBrushless);

    wristEncoder = wristMotor.getAbsoluteEncoder();

    wristPID = wristMotor.getPIDController();
    wristPID.setFeedbackDevice(wristEncoder);

    wristPID.setP(WristConstants.kWristP);
    wristPID.setI(WristConstants.kWristI);
    wristPID.setD(WristConstants.kWristD);
    wristPID.setFF(WristConstants.kWristFF);
    wristPID.setSmartMotionAllowedClosedLoopError(
      WristConstants.kwristAngleTolerance,
      0
    );

    wristPID.setPositionPIDWrappingEnabled(true);
    wristPID.setPositionPIDWrappingMinInput(
      WristConstants.kEncoderPositionPIDMinInput
    );
    wristPID.setPositionPIDWrappingMaxInput(
      WristConstants.kEncoderPositionPIDMaxInput
    );
    wristPID.setOutputRange(-1, 1);

    wristMotor.burnFlash();
  }

  public void setWristAngle(double target) {
    target -= WristConstants.kWristOffsetAngle;
    if (target < 0) {
      target = 0;
    }
    wristTargetAngle = target;
    wristTarget = target / 360;
    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kPosition);
  }

  public void wristStop() {
    wristMotor.stopMotor();
  }

  public double getWristAngle() {
    return wristEncoder.getPosition() * 360 + WristConstants.kWristOffsetAngle;
  }

  public double getWristEncoderVal() {
    return wristEncoder.getPosition();
  }

  public void home() {
    if (wristEncoder.getPosition() > 0) {
      wristMotor.set(-0.05);
    }
  }

  public WristState getState() {
    return state;
  }

  private void updateWristState() {
    if (
      Math.abs(getWristEncoderVal() - wristTarget) >
      WristConstants.kwristAngleTolerance
    ) {
      state = WristState.MOVING;
    } else {
      state = WristState.AT_POSITION;
    }
  }

  public void debugValues() {
    SmartDashboard.putNumber("Encoder Val", getWristEncoderVal());
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putNumber("Wrist Target", wristTarget);
    SmartDashboard.putNumber("Wrist Target Angle", wristTargetAngle);
    SmartDashboard.putString("Wrist State", getState().toString());
    SmartDashboard.putNumber(
      "Wrist Encoder Offset",
      wristEncoder.getZeroOffset()
    );
    SmartDashboard.putNumber("Motor Speed", wristMotor.get());
    // SmartDashboard.putBoolean("Wrist Home Endstop", wristEndstop.get());
  }

  public void competitionValues() {}

  @Override
  public void periodic() {
    updateWristState();
    debugValues();
    //competitionValues();
  }

  public enum WristState {
    MOVING,
    AT_POSITION,
  }
}
