// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SubsystemCommands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAtTagOdometry extends PIDCommand {
  /** Creates a new AimAtTag. */
  public AimAtTagOdometry(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera, int tagID) {
    super(
        // The controller that the command will use
        new PIDController(0.5, 0, 0),
        // Returns the current heading (the measurement)
        () -> {
          return swerveSubsystem.getHeading().getRadians();
        },
        // Returns the desired heading for the robot (the setpoint)
        () -> {

          Pose2d robotPose = swerveSubsystem.getPose();
          AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
          Pose3d desiredTag = aprilTagFieldLayout.getTagPose(tagID).orElseThrow();

          Translation2d robotPosition = robotPose.getTranslation();
          Translation3d tagPosition = desiredTag.getTranslation();

          // get angle between vectors, factor in rotation
          // theta = atan2(w2 * v1 - w1 * v2, w1 * v1 - w2 * v2)
          // w is target vector, w1 is target x, w2 is target y
          // v is current vector, v1 is current x, v2 is current y
          double angle = Math.atan2(
              tagPosition.getY() * robotPose.getX() - tagPosition.getX() * robotPosition.getY(),
              tagPosition.getX() * robotPosition.getX() - tagPosition.getY() * robotPosition.getY());

          while (angle > Math.PI) {
            angle -= 2 * Math.PI;
          }

          return angle + swerveSubsystem.getHeading().getRadians();
        },
        // Turn the robot to the desired angle
        output -> {
          System.out.println(swerveSubsystem.getHeading().getRadians() + " -- > " + output);
          ChassisSpeeds speeds = swerveSubsystem.getTargetSpeeds(0, 0,
              new Rotation2d(output + Math.PI));
          swerveSubsystem.drive(speeds.times(0.5));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(swerveSubsystem);
    getController().setTolerance(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
