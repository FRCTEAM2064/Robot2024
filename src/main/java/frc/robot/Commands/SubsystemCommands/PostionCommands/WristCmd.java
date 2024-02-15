package frc.robot.Commands.SubsystemCommands.PostionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist;

public class WristCmd extends Command {

  private Wrist wrist;
  private double angle;

  public WristCmd(Wrist wrist, double angle) {
    this.wrist = wrist;
    this.angle = angle;
  }

  @Override
  public void execute() {
    wrist.setWristAngle(angle);
  }
}
