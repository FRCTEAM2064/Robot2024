package frc.robot.Commands.SubsystemCommands;

import java.lang.reflect.InaccessibleObjectException;

import javax.lang.model.util.ElementScanner14;

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

  private boolean executingHandoff = false;

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
  public void initialize() {
    if (!elevator.elevatorHasZeroed) {
      this.cancel();
    }

  }

  @Override
  public void execute() {
    
    if(executingHandoff == false){
    intake.intakeTimer.start();
    elevator.setElevatorHeight(ElevatorConstants.kElevatorHandoffHeight);
    wrist.setWristAngle(WristConstants.kWristHandoffAngle);
    intake.setIntakeAngle(IntakeConstants.kIntakeHandOffAngle);
    executingHandoff = true;
    }
    if (
      intake.getState() == IntakeState.AT_POSITION &&
      elevator.getState() == ElevatorState.AT_POSITION &&
      wrist.getState() == WristState.AT_POSITION
    ) {
      shooter.intake();
      intake.outtake();
    }
  }

  @Override
  public boolean isFinished() {
    if(shooter.hasGamePiece || intake.intakeTimer.get() > 3){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.setIntakeAngle(IntakeConstants.kIntakeHome);
    wrist.setWristAngle(WristConstants.kWristHome);
    elevator.setElevatorHeight(4);
    shooter.feederStop();
    executingHandoff = false;
    intake.intakeTimer.stop();
    intake.intakeTimer.reset();

  }
}
