// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.ElevatorConstants;
import frc.robot.Constants.Constants.WristConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Wrist.WristState;


public class ScoreAmpCmd extends Command {
  /** Creates a new ScoreAmpCmd. */
  Elevator elevator;
  Shooter shooter;
  Wrist wrist;
  private boolean hasFinished = false;
  Timer wristTimer = new Timer();
  public ScoreAmpCmd(Elevator elevator, Wrist wrist, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.wrist = wrist;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevatorHeight(9);
    wrist.setWristAngle(110);
    wristTimer.start();
    if(elevator.getState() == ElevatorState.AT_POSITION && wrist.getState() == WristState.AT_POSITION){
      shooter.shoot();
      if(shooter.getState() == ShooterState.STOP && wristTimer.get() >= 3){
        hasFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setWristAngle(WristConstants.kWristHome);
    elevator.setElevatorHeight(ElevatorConstants.kElevatorHome);
    wristTimer.stop();
    wristTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasFinished;
  }
}
