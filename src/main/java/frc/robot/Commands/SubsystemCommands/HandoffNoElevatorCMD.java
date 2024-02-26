// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.WristConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Wrist.WristState;

public class HandoffNoElevatorCMD extends Command {
  /** Creates a new HandoffNoElevatorCMD. */
  private Shooter shooter;
  private Intake intake;
  private Wrist wrist;
  private boolean isFinished = false;
  public HandoffNoElevatorCMD(Shooter shooter, Intake intake, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.intake = intake;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setWristAngle(WristConstants.kWristHandoffAngle);
    intake.setIntakeAngle(IntakeConstants.kIntakeHandOffAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wrist.getState() == WristState.AT_POSITION && intake.getState() == IntakeState.AT_POSITION){
      if (!shooter.hasGamePeice){
        intake.outtake();
        shooter.feed();
      } else {
        isFinished = true;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.setIntakeAngle(0);
    wrist.setWristAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return isFinished;
  }
}
