// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.WristConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Wrist.WristState;
import frc.robot.Subsystems.Wrist;

public class ShootFromIntakeCmd extends Command {

  /** Creates a new ShootFromIntakeCmd. */
  private Intake intake;
  private Shooter shooter;
  private boolean didStart = false; 
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
   System.out.println("Inside Initialize of Shoot from Intake");
   didStart = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setWristAngle(10);
    // shooter.shoot();
    if (shooter.getState() == ShooterState.FEEDING) {
      intake.outtake();
    }
    // this is checking, so we only start once
    else if(didStart == false){
      shooter.shoot();
      didStart = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    // wrist.setWristAngle(WristConstants.kWristHome);
    // intake.setIntakeAngle(IntakeConstants.kIntakeHome);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.getState() == ShooterState.STOP) {
      return true;
    }
    return false;
  }
}
