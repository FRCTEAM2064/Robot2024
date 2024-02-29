package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;

public class IntakeFloorCmd extends Command {

  private Intake intake;
  private Elevator elevator;

  public IntakeFloorCmd(Intake intake, Elevator elevator) {
    this.intake = intake;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    elevator.setElevatorHeight(5);
  }


  @Override
  public void execute() {
    if (!intake.hasGamePiece) {
      intake.intake();
      intake.setIntakeAngle(IntakeConstants.kIntakeFloorAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return intake.hasGamePiece;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.setIntakeAngle(IntakeConstants.kIntakeHome);
  }
}
