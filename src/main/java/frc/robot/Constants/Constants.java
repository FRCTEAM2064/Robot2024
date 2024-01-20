package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static class OIConstants {

    public static final int kDriverControllerPort = 0;

    public static final double kDeadband = 0.1;

    public static final int kXboxAButton = 1;
    public static final int kXboxBButton = 2;
    public static final int kXboxXButton = 3;
    public static final int kXboxYButton = 4;
    public static final int kXboxLeftBumper = 5;
    public static final int kXboxRightBumper = 6;
    public static final int kXboxBackButton = 7;
    public static final int kXboxStartButton = 8;
    public static final int kXboxLeftStickButton = 9;
    public static final int kXboxRightStickButton = 10;
    public static final int kXboxLeftTriggerAxis = 2;
    public static final int kXboxRightTriggerAxis = 3;
    public static final int kXboxLeftXAxis = 0;
    public static final int kXboxLeftYAxis = 1;
    public static final int kXboxRightXAxis = 4;
    public static final int kXboxRightYAxis = 5;
  }

  public static class DriveConstants {

    public static final boolean kPidgeonGyro = true;

    public static final double kTrackWidth = Units.inchesToMeters(30);
    public static final double kWheelBase = Units.inchesToMeters(30);

    public static final double kMaxAcceleration = 3;
    public static final double kMaxAngularAcceleration = 3;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    public static double kVisionRotationP;

    public static double kVisionTranslationP;
  }

  public static class VisionConstants {

    public static final double kFrontCameraPitch = 0;
    public static final Transform3d kFrontRobotToCam = new Transform3d(
      new Translation3d(0.5, 0.5, 0.5),
      new Rotation3d(0, 0, 0)
    );

    public static final String kFrontCamName = "Camera 1";
  }
}
