package frc.robot.Commands.AutoCommnads;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import java.io.File;

import javax.xml.crypto.dsig.TransformException;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveDistance extends TrapezoidProfileCommand {

  public DriveDistance(double distance, SwerveSubsystem drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        new TrapezoidProfile(new TrapezoidProfile.Constraints(drivebase.maximumSpeed, 1)),
        (state) -> {
          double setpoint = state.position;
          System.out.println("Drive distance setpoint, " + setpoint);
          Translation2d offset = new Translation2d(setpoint, drivebase.getHeading());
          // Pose2d maxPose = new Pose2d(initalPose.getTranslation().plus(offset),
          // drivebase.getHeading());
          drivebase.drive(offset, 0, false);
        },
        () -> new TrapezoidProfile.State(distance, 0), // desired end goal position
        TrapezoidProfile.State::new, // current position
        drivebase);

  }
}