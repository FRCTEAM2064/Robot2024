package frc.robot.Commands.SubsystemCommands.PostionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Intake;

public class IntakeCmd extends InstantCommand {

  private Intake intake;
  private double angle;

  public IntakeCmd(Intake intake, double angle) {
    this.intake = intake;
    this.angle = angle;
  }

  @Override
  public void execute() {
    intake.setIntakeAngle(angle);
  }
}
