package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax wristMotor;
  private CANSparkMax intakeMotor;

  private RelativeEncoder wristEncoder;

  private IntakeState state = IntakeState.IDLE;
  private double wristPosition;
  private double wristTarget;

  public Intake() {
    wristMotor =
      new CANSparkMax(IntakeConstants.kWristMotorID, MotorType.kBrushless);
    intakeMotor =
      new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

    wristEncoder =
      wristMotor.getAlternateEncoder(Constants.kThroughBoreEncoderRev);
  }

  public enum IntakeState {
    IDLE,
    MOVING,
    AT_POSITION,
    STOPPED,
  }
}
