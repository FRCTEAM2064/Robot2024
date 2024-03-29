package frc.robot.Constants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

  public static final int kThroughBoreEncoderRev = 8192;

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
      new Translation3d(0.5, 0.5, 0.5),
      new Rotation3d(0, 0, 0)
    );

    public static final String kFrontCamName = "Camera 1";
  }

  public static class ElevatorConstants {
    public static final int kElevatorEndstopDIO = 0;

    public static final int kLeaderMotorID = 25;
    public static final int kFollowerMotorID = 26;

    public static final int kElevatorMin = 0;
    public static final int kElevatorMax = 100;
    public static final int kElevatorClimb = 0;

    public static final double kElevatorP = 1;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;

    public static final double kElevatorRatio = 1/25;
    public static final double kElevatorChainPitch = 0.25;
    public static final int kElevatorSprocket = 25; 

    public static final double kElevatorHeightTolerance = 0.5;

    public static final double kElevatorHandoffHeight = 0;
  }

  public static class WristConstants {
    public static final int kWristEndstopDIO = 2;

    public static final double kWristHandoffAngle = 60;

    public static final int kWristMotorID = 27;
    public static final double kwristAngleTolerance = 0.002;

    public static final double kWristP = 4;
    public static final double kWristI = 0;
    public static final double kWristD = 0;

    
    public static final double kEncoderPositionPIDMinInput = 0;
    public static final double kEncoderPositionPIDMaxInput = 1;
  
  }

  public static class ShooterConstants {

    public static final int kLeaderMotorID = 35;
    public static final int kFollowerMotorID = 36;
    public static final int kFeederMotorID = 37;

    public static final double kShooterTargetSpeed = 6500.0;
    public static final double kFeedDuration = 2.0;
    public static final int kHasGamePieceLimitDIO = 4;

  }

  public static class IntakeConstants {

    
    public static final int kHasGamePieceLimitDIO = 3;
    public static final int kHomePositionLimitDIO = 1;

    public static final int kIntakePivotMotorID = 45;
    public static final int kIntakeMotorID = 46;

    public static final double kIntakeConversion = 16 / 24;
    public static final double kIntakeAngleTolerance = 0.5;
    public static final double kIntakeP =   8;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;

    public static final double kIntakeFloorAngle = 10;
    public static final double kIntakeStationAngle = 10;
    public static final double kIntakeHandOffAngle = 0;
    public static final double kIntakeHome = 0;

  }

  public static class AutonConstants {

    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(
      0.7,
      0,
      0
    );
  }
}
