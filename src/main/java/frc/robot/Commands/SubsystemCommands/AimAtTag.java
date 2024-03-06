// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SubsystemCommands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAtTag extends PIDCommand {
  /** Creates a new AimAtTag. */
  public AimAtTag(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera) {
    super(
        // The controller that the command will use
        new PIDController(0.5, 0, 0),
        // This should return the measurement
        () -> {
          if (!photonCamera.getLatestResult().hasTargets()) {
            System.out.println("Couldn't find target");
            return 0.0;
          }
          PhotonTrackedTarget latestResult = photonCamera.getLatestResult().getBestTarget();
          if (latestResult != null) {
            return latestResult.getYaw();
          }
          else {
            return 0.0;
          }
        },
        // This should return the setpoint (can also be a constant)
        0.0,
        // This uses the output
        output -> {
          System.out.println("Output: " + output);
          swerveSubsystem.drive(new Translation2d(0, 0), output / 5, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(swerveSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
