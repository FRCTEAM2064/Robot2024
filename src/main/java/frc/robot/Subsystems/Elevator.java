package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
  }

  public ElevatorState getElevatorState() {
    return state;
  }

  public void setElevatorHeight(double height) {
    targetHeight = height;
  }

  public double getElevatorHeight() {
    return elevatorEncoder.getPosition();
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

  @Override
  public void periodic() {
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
