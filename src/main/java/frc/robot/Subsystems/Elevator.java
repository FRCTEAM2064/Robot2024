package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private CANSparkMax elevatorLeaderMotor;
  private CANSparkMax elevatorFollowerMotor;
  private RelativeEncoder elevatorEncoder;

  private SparkPIDController elevatorLeaderPID;

  private ElevatorState state = ElevatorState.AT_POSITION;

  private double targetHeight;

  private DigitalInput elevatorEndstop;

  public Elevator() {
    elevatorLeaderMotor =
      new CANSparkMax(
        Constants.ElevatorConstants.kLeaderMotorID,
        MotorType.kBrushless
      );
    elevatorFollowerMotor =
      new CANSparkMax(
        Constants.ElevatorConstants.kFollowerMotorID,
        MotorType.kBrushless
      );

    elevatorLeaderMotor.getEncoder();

    elevatorFollowerMotor.follow(elevatorLeaderMotor, true);

    elevatorEncoder = elevatorLeaderMotor.getEncoder();

    elevatorLeaderPID = elevatorLeaderMotor.getPIDController();

    elevatorLeaderPID.setP(ElevatorConstants.kElevatorP);
    elevatorLeaderPID.setI(ElevatorConstants.kElevatorI);
    elevatorLeaderPID.setD(ElevatorConstants.kElevatorD);

    elevatorLeaderPID.setSmartMotionAllowedClosedLoopError(ElevatorConstants.kElevatorHeightTolerance, 0);

    elevatorEndstop = new DigitalInput(ElevatorConstants.kElevatorEndstopDIO);
  }

  public ElevatorState getState() {
    return state;
  }

  public void zeroElevator(){
    elevatorEncoder.setPosition(0);
  }

  /*
   * Current height in inches
   */
  public double getElevatorHeight() {
    double encoderCountsPerRevolution = elevatorEncoder.getCountsPerRevolution();

    double sprocketPitchDiameter = (ElevatorConstants.kElevatorChainPitch * ElevatorConstants.kElevatorSprocket) / Math.PI;
    double sprocketCircumference = Math.PI * sprocketPitchDiameter;

    double distancePerCount = sprocketCircumference / (encoderCountsPerRevolution * ElevatorConstants.kElevatorRatio);

    return elevatorEncoder.getPosition() * distancePerCount;
}


  public void setElevatorHeight(double height) {
    targetHeight = height;
  }


  private void updateElevatorState() {
    if (
      Math.abs(getElevatorHeight() - targetHeight) >
      ElevatorConstants.kElevatorHeightTolerance
    ) {
      state = ElevatorState.MOVING;
    } else {
      state = ElevatorState.AT_POSITION;
    }
  }

  private void endStopProtection(){
    if (elevatorEndstop.get()){
      zeroElevator();
      elevatorLeaderMotor.stopMotor();
      targetHeight = 0;
    }
  }

  public void home(){
    if (elevatorEncoder.getPosition() > 0){
      elevatorLeaderMotor.set(-0.05);
    }
  }

  public void debugValues(){
    SmartDashboard.putNumber("Target Height Encoder", targetHeight);
  }

  @Override
  public void periodic() {
    endStopProtection();
    elevatorLeaderPID.setReference(
      targetHeight,
      CANSparkMax.ControlType.kPosition
    );
    updateElevatorState();
  }

  public enum ElevatorState {
    MOVING,
    AT_POSITION,
  }
}
