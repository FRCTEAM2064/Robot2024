package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.ElevatorConstants;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.WristConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Wrist.WristState;

public class HandoffCmd extends Command {

  private final Wrist wrist;
  private final Shooter shooter;
  private final Intake intake;
  private final Elevator elevator;

  private Boolean intakeInPos = false;
  private Boolean wristInPos = false;
  private Boolean elevatorInPos = false;
  private boolean isFinished = false;

  public HandoffCmd(
    Wrist wrist,
    Shooter shooter,
    Intake intake,
    Elevator elevator
  ) {
    this.wrist = wrist;
    this.shooter = shooter;
    this.intake = intake;
    this.elevator = elevator;
  }

  @Override
  public void execute() {
    if (!intake.hasGamePiece) {
      cancel();
    }

    if (!intakeInPos) {
      intake.setIntakeAngle(IntakeConstants.kIntakeHandOffAngle);
    } else {
      wrist.setWristAngle(WristConstants.kWristHandoffAngle);
      wristInPos = wrist.getState() == WristState.AT_POSITION;
    }

    if (wristInPos) {
      elevator.setElevatorHeight(ElevatorConstants.kElevatorHandoffHeight);
      elevatorInPos = elevator.getState() == ElevatorState.AT_POSITION;
    }

    if (intakeInPos && wristInPos && elevatorInPos) {
      isFinished = true;
    }

    intakeInPos = intake.getState() == IntakeState.AT_POSITION;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      shooter.intake();
      intake.outtake();
    }
  }
}
