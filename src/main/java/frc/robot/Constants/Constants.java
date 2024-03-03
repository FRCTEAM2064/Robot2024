package frc.robot.Constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final int kThroughBoreEncoderRev = 8192;

  public final static double kSpeakerHeight = 80.0;
  public final static double kSpeakerYOffset = 218.5;
  public final static double kSpeakerXOffset = 9;

  public static class OIConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

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

  public static class VisionConstants {

    public static final double kFrontCameraPitch = 0;
    public static final Transform3d kFrontRobotToCam = new Transform3d(
      new Translation3d(11.75, 4.5, 17.5),
      new Rotation3d(15, 0, 0)
    );

    public static final String kFrontCamName = "photonCam";
  }

  public static class ElevatorConstants {

    public static final int kElevatorEndstopDIO = 0;

    public static final int kLeaderMotorID = 23;
    public static final int kFollowerMotorID = 24;

    public static final int kElevatorMin = 0;
    public static final int kElevatorMax = 100;
    public static final int kElevatorClimb = 0;

    public static final double kElevatorP = 1;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;

    public static final double kElevatorRatio = 4.5;

    public static final double kElevatorHeightTolerance = 0.5;

    public static final double kElevatorHandoffHeight = 5;

    public static final double kElevatorHome = 2;

    public static final double kElevatorHomedHeight = 24;
  }

  public static class WristConstants {

    public static final double kWristHandoffAngle = 35;

    public static final int kWristMotorID = 25;
    public static final double kwristAngleTolerance = 0.02;

    public static final double kWristP = 4;
    public static final double kWristI = 0;
    public static final double kWristD = 0;

    public static final double kEncoderPositionPIDMinInput = 0;
    public static final double kEncoderPositionPIDMaxInput = 1;

    public static final double kWristFF = 0.1;

    public static final double kWristOffsetAngle = 25;

    public static final double kWristHome = 25;
  }

  public static class ShooterConstants {

    public static final int kFeederMotorID = 26;
    public static final int kLeaderMotorID = 27;
    public static final int kFollowerMotorID = 28;

    public static final double kShooterTargetSpeed = 6200.0;
    public static final double kFeedDuration = 1.5;
    public static final int kFeederMotorPDHPos = 9;
    public static final double kFeederMotorDrawLimit = 5.5;

    public static final int kHasGamePieceDIO = 1; 
  }

  public static class IntakeConstants {

    public static final int kHasGamePieceLimitDIO = 7;

    public static final int kIntakePivotMotorID = 21;
    public static final int kIntakeMotorID = 22;

    public static final double kIntakeAngleTolerance = 0.05;
    public static final double kIntakeP = 1.5;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;

    public static final double kIntakeFloorAngle = 300;
    public static final double kIntakeStationAngle = 120;
    public static final double kIntakeHandOffAngle = 47.5;
    public static final double kIntakeHome = 0;

    public static final double kIntakeOffset = .05;
    public static final double kIntakeAcceleration = 0.5;
  }

  public static class AutonConstants {

    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(
      0.7,
      0,
      0
    );
  }

  public static final double targetWidth =
  Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

// See
// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
// page 197
public static final double targetHeight =
  Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

// See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
// pages 4 and 5
public static final double kFarTgtXPos = Units.feetToMeters(54);
public static final double kFarTgtYPos =
  Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
public static final double kFarTgtZPos =
  (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

public static final Pose3d kFarTargetPose =
  new Pose3d(
          new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));}
