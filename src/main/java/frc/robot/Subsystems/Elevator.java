package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Elevator extends SubsystemBase {

  private CANSparkMax elevatorLeaderMotor;
  private CANSparkMax elevatorFollowerMotor;
  private CANSparkMax wristMotor;

  private RelativeEncoder elevatorLeaderEncoder;
  private RelativeEncoder wristEncoder;

  private SparkPIDController elevatorLeaderPID;
  private SparkPIDController wristPID;

  public Elevator() {
    elevatorLeaderMotor =
      new CANSparkMax(
        Constants.ElevatorConstants.kLeaderMotorID,
        CANSparkLowLevel.MotorType.kBrushed
      );
    elevatorFollowerMotor =
      new CANSparkMax(
        Constants.ElevatorConstants.kFollowerMotorID,
        CANSparkLowLevel.MotorType.kBrushed
      );

    elevatorLeaderEncoder = elevatorLeaderMotor.getEncoder();

    wristEncoder =
      wristMotor.getAlternateEncoder(Constants.kThroughBoreEncoderRev);

    elevatorFollowerMotor.follow(elevatorLeaderMotor, true);

    elevatorLeaderPID = elevatorLeaderMotor.getPIDController();
    wristPID = wristMotor.getPIDController();

    wristPID.setFeedbackDevice(wristEncoder);

    elevatorLeaderPID.setP(Constants.ElevatorConstants.kPLeaderElevator);
    elevatorLeaderPID.setI(Constants.ElevatorConstants.kILeaderElevator);
    elevatorLeaderPID.setD(Constants.ElevatorConstants.kDLeaderElevator);
    elevatorLeaderPID.setIZone(Constants.ElevatorConstants.kIzLeaderElevator);
    elevatorLeaderPID.setFF(Constants.ElevatorConstants.KFFLeaderElevator);
    elevatorLeaderPID.setOutputRange(
      Constants.ElevatorConstants.kMinOutLeaderElevator,
      Constants.ElevatorConstants.kMaxOutLeaderElevator
    );

    elevatorLeaderPID.setSmartMotionMaxVelocity(
      Constants.ElevatorConstants.kMaxVelLeaderElevator,
      0
    );
    elevatorLeaderPID.setSmartMotionMinOutputVelocity(0, 0);
    elevatorLeaderPID.setSmartMotionMaxAccel(
      Constants.ElevatorConstants.kmMaxAccLeaderElevator,
      0
    );
    elevatorLeaderPID.setSmartMotionAllowedClosedLoopError(0, 0);
  }

  public void stopElevator() {
    elevatorLeaderMotor.set(0);
  }

  public void setElevatorSpeed(double speed) {
    elevatorLeaderMotor.set(speed);
  }

  /**
   * Sets the Elevator height using a pid tuner and elevator position encoders
   * @param height desired elevator height in inches
   */
  public void setElevatorHeight(double height) {
    // set the elevator to a specefied height in inches
  }
}
