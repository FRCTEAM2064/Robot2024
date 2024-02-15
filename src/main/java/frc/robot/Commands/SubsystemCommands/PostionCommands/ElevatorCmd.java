package frc.robot.Commands.SubsystemCommands.PostionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class ElevatorCmd extends Command {

  private Elevator elevator;
  private double height;

  public ElevatorCmd(Elevator elevator, double height) {
    this.elevator = elevator;
    this.height = height;
  }

  @Override
  public void execute() {
    elevator.setElevatorHeight(height);
  }
}
