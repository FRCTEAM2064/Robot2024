// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Shooter.ShooterState;

public class ShootFromIntakeCmd extends Command {
  /** Creates a new ShootFromIntakeCmd. */
  private Intake intake;
  private Shooter shooter;
  private Wrist wrist;
  public ShootFromIntakeCmd(Intake intake, Shooter shooter, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;
    this.wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.shoot();
    wrist.setWristAngle(14);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getState() == ShooterState.FEEDING){
      intake.outtake();
    }
    if(shooter.getState() == ShooterState.STOP){
      end(isFinished());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    //wrist.setWristAngle(25);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
