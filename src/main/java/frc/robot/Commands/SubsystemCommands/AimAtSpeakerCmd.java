// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ElevatorConstants;
import frc.robot.Constants.Constants.WristConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Wrist;

public class AimAtSpeakerCmd extends Command {
  /** Creates a new AimAtSpeakerCmd. */
  Elevator elevator;
  Wrist wrist;
  SwerveSubsystem drive;
  double calcedSetAngle;
  public AimAtSpeakerCmd(Elevator elevator, Wrist wrist, SwerveSubsystem drive) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double zDistance = (Constants.kSpeakerHeight - (elevator.getElevatorHeight() + ElevatorConstants.kElevatorHomedHeight));
    double xDistance = drive.distanceToSpeaker;
    double angle = Math.tan(zDistance / xDistance);
    wrist.setWristAngle(90 - angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setWristAngle(WristConstants.kWristHome);
  }

}
