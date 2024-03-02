package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
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

  public CANSparkMax elevatorLeaderMotor;
  private CANSparkMax elevatorFollowerMotor;
  private RelativeEncoder elevatorEncoder;

  private SparkPIDController elevatorLeaderPID;

  private ElevatorState state = ElevatorState.AT_POSITION;

  private double targetHeight;

  private DigitalInput elevatorEndstop;
  public boolean elevatorHasZeroed;

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



    elevatorFollowerMotor.follow(elevatorLeaderMotor, true);

    elevatorEncoder = elevatorLeaderMotor.getEncoder();

    elevatorEncoder.setPositionConversionFactor(1);

    elevatorLeaderPID = elevatorLeaderMotor.getPIDController();

    elevatorLeaderPID.setP(ElevatorConstants.kElevatorP);
    elevatorLeaderPID.setI(ElevatorConstants.kElevatorI);
    elevatorLeaderPID.setD(ElevatorConstants.kElevatorD);

    elevatorLeaderPID.setSmartMotionAllowedClosedLoopError(
      ElevatorConstants.kElevatorHeightTolerance,
      0
    );

    elevatorEndstop = new DigitalInput(ElevatorConstants.kElevatorEndstopDIO);
    debugValues();

    elevatorEncoder.setPositionConversionFactor(1);

    elevatorLeaderMotor.setSmartCurrentLimit(40);
    elevatorFollowerMotor.setSmartCurrentLimit(40);

    elevatorLeaderMotor.setIdleMode(IdleMode.kBrake);
    elevatorFollowerMotor.setIdleMode(IdleMode.kBrake);

    elevatorHasZeroed = false;
    // zeroElevator();
  }

  public void elevatorStop() {
    elevatorLeaderMotor.stopMotor();
  }

  public ElevatorState getState() {
    return state;
  }

  public void zeroElevator() {
    elevatorHasZeroed = true;
    elevatorEncoder.setPosition(0);
    elevatorLeaderMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
  }

  /*
   * Current height in inches
   */
  public double getElevatorHeight() {
    return elevatorEncoder.getPosition() / ElevatorConstants.kElevatorRatio;
  }

  public void setElevatorHeight(double height) {
    if (elevatorHasZeroed) {
      targetHeight = height * ElevatorConstants.kElevatorRatio;
      elevatorLeaderPID.setReference(targetHeight, ControlType.kPosition);
    }
  }

  private void updateElevatorState() {
    if (
      Math.abs(elevatorEncoder.getPosition() - targetHeight) >
      ElevatorConstants.kElevatorHeightTolerance
    ) {
      state = ElevatorState.MOVING;
    } else {
      state = ElevatorState.AT_POSITION;
    }
  }

  //ADD MAX HEIGHT?
  private void endStopProtection() {
    if (!elevatorEndstop.get() && !elevatorHasZeroed){
      zeroElevator();
      elevatorLeaderMotor.stopMotor();
      targetHeight = 0;
    }
  }

  public void home() {
    elevatorLeaderMotor.set(-0.1);
  }

  public void debugValues() {
    SmartDashboard.putNumber("Target Height Encoder", targetHeight);
    SmartDashboard.putNumber(
      "Elevator Encoder val",
      elevatorEncoder.getPosition()
    );
    SmartDashboard.putBoolean("Elevator Switch", elevatorEndstop.get());
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    SmartDashboard.putNumber("Elevator Target Height", targetHeight);
    SmartDashboard.putString("Elevator State", state.toString());
  }

  @Override
  public void periodic() {
    endStopProtection();
    updateElevatorState();
    debugValues();
  }

  public enum ElevatorState {
    MOVING,
    AT_POSITION,
  }
}
