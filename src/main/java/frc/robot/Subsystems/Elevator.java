package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private CANSparkMax elevatorLeaderMotor;
  private CANSparkMax elevatorFollowerMotor;

  private SparkPIDController elevatorLeaderPID;

  private ElevatorState state = ElevatorState.IDLE;
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

    elevatorLeaderPID = elevatorLeaderMotor.getPIDController();

    elevatorLeaderPID.setP(ElevatorConstants.kPLeaderElevator);
    elevatorLeaderPID.setI(ElevatorConstants.kILeaderElevator);
    elevatorLeaderPID.setD(ElevatorConstants.kDLeaderElevator);
    elevatorLeaderPID.setIZone(ElevatorConstants.kIzLeaderElevator);
    elevatorLeaderPID.setFF(ElevatorConstants.KFFLeaderElevator);
    elevatorLeaderPID.setOutputRange(
      ElevatorConstants.kMinOutLeaderElevator,
      ElevatorConstants.kMaxOutLeaderElevator
    );

    elevatorLeaderPID.setSmartMotionMaxVelocity(
      ElevatorConstants.kMaxVelLeaderElevator,
      0
    );
    elevatorLeaderPID.setSmartMotionMinOutputVelocity(0, 0);
    elevatorLeaderPID.setSmartMotionMaxAccel(
      ElevatorConstants.kMaxAccLeaderElevator,
      0
    );
    elevatorLeaderPID.setSmartMotionAllowedClosedLoopError(0, 0);
  }

  public void stopElevator() {
    elevatorLeaderMotor.set(0);
    state = ElevatorState.STOPPED;
  }

  public double getElevatorHeight() {
    // needs to return the height of the elevator in inches
    return 0;
  }

  public void setElevatorHeight(double height) {
    targetHeight = height;
    state = ElevatorState.MOVING;
    // set the pid tuning here
  }

  public void periodic() {
    updateElevatorState();
  }

  public void updateElevatorState() {
    switch (state) {
      case IDLE:
        break;
      case MOVING:
        if (
          Math.abs(getElevatorHeight() - targetHeight) <
          ElevatorConstants.kElevatorHeightTolerance
        ) {
          state = ElevatorState.AT_TARGET;
        }
        break;
      case AT_TARGET:
        // hold position
        break;
      case STOPPED:
        stopElevator();
        state = ElevatorState.IDLE;
        break;
    }
  }

  public enum ElevatorState {
    IDLE,
    MOVING,
    AT_TARGET,
    STOPPED,
  }
}
