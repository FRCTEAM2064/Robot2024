package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public class Drivetrain extends SubsystemBase {

  private final SwerveModule backLeftModule = new SwerveModule("BL");
  private final SwerveModule frontLeftModule = new SwerveModule("FL");
  private final SwerveModule backRightModule = new SwerveModule("BR");
  private final SwerveModule frontRightModule = new SwerveModule("FR");
  private Pigeon2 gyro = new Pigeon2(60);


  public PhotonCameraWrapper FrontCam;

  private final SwerveDrivePoseEstimator positionEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    getRotation2d(),
    getModulePositions(),
    new Pose2d()
  );

  

  public Drivetrain() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
      } catch (Exception e) {}
    })
      .start();

    zeroHeading();

    FrontCam =
      new PhotonCameraWrapper(
        VisionConstants.kFrontCamName,
        VisionConstants.kFrontRobotToCam
      );

  

AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.5, 0.0, 0.0), // Rotation PID constants
            1, // Max module speed, in m/s
            0.381, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this::color, 
        this // Reference to this subsystem to set requirements
    );      
  }

  
  public Pose2d getPose() {
     return positionEstimator.getEstimatedPosition();
  }

  public boolean color() {
    return false;
  }

  public void resetPose(Pose2d robotPose) {
    positionEstimator.resetPosition(robotPose.getRotation(), getModulePositions(), robotPose);
  }

  public ChassisSpeeds getRobotRelativSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[] {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    };
    ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    return speeds;
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    // Convert robot-relative chassis speeds to module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);

    // Set the state for each module
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
}



  public void zeroHeading() {
    gyro.reset();
  }

  public void resetOdometry(Pose2d pose) {
    positionEstimator.update(getRotation2d(), getModulePositions());
    positionEstimator.resetPosition(
      getRotation2d(),
      getModulePositions(),
      pose
    );
  }

  public double getHeading() {
    return gyro.getAngle();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getModulePosition(),
      frontRightModule.getModulePosition(),
      backLeftModule.getModulePosition(),
      backRightModule.getModulePosition(),
    };


    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putNumber(
      "FL Cancoder",
      frontLeftModule.getAbsolutePositionDegrees()
    );
    SmartDashboard.putNumber(
      "FR Cancoder",
      frontRightModule.getAbsolutePositionDegrees()
    );
    SmartDashboard.putNumber(
      "BL Cancoder",
      backLeftModule.getAbsolutePositionDegrees()
    );
    SmartDashboard.putNumber(
      "BR Cancoder",
      backRightModule.getAbsolutePositionDegrees()
    );

    positionEstimator.update(getRotation2d(), getModulePositions());
  }

  public void stopModules() {
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  public double getLowestSpeed() {
    double[] speeds = new double[4];
    speeds[0] = frontLeftModule.getSpeed();
    speeds[1] = frontRightModule.getSpeed();
    speeds[2] = backLeftModule.getSpeed();
    speeds[3] = backRightModule.getSpeed();
    double lowest = speeds[0];
    for (int i = 1; i < speeds.length; i++) {
      if (speeds[i] < lowest) {
        lowest = speeds[i];
      }
    }
    return lowest;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      getLowestSpeed()
    );
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);

    SmartDashboard.putNumber(
      "FL Desired State",
      desiredStates[0].angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "FR Desired State",
      desiredStates[1].angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "BL Desired State",
      desiredStates[2].angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "BR Desired State",
      desiredStates[3].angle.getDegrees()
    );

    SmartDashboard.putNumber(
      "FL Turn Encoder Val",
      Math.toDegrees(frontLeftModule.getTurnPosition())
    );
    SmartDashboard.putNumber(
      "FR Turn Encoder Val",
      Math.toDegrees(frontRightModule.getTurnPosition())
    );
    SmartDashboard.putNumber(
      "BL Turn Encoder Val",
      Math.toDegrees(backLeftModule.getTurnPosition())
    );
    SmartDashboard.putNumber(
      "BR Turn Encoder Val",
      Math.toDegrees(backRightModule.getTurnPosition())
    );
}

  public void updateOdometry() {
    positionEstimator.update(getRotation2d(), getModulePositions());

    Optional<EstimatedRobotPose> result = FrontCam.getEstimatedGlobalPose(
      positionEstimator.getEstimatedPosition()
    );

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      positionEstimator.addVisionMeasurement(
        camPose.estimatedPose.toPose2d(),
        camPose.timestampSeconds
      );
    }
  }

  public void trackTargetIDRotation(Integer targetID) {
    PhotonPipelineResult result = FrontCam.photonCamera.getLatestResult();

    if (
      result.hasTargets() &&
      (result.getBestTarget().getFiducialId() == targetID || targetID == 99)
    ) {
      double yaw = result.getBestTarget().getYaw();
      double yawRate = result.getBestTarget().getYaw();
      double yawSetpoint = yaw + yawRate * 0.1;
      double yawError = yawSetpoint - getHeading();
      double yawOutput = yawError * DriveConstants.kVisionRotationP;
      setModuleStates(
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(0, 0, yawOutput)
        )
      );
    } else {
      setModuleStates(
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(0, 0, 0)
        )
      );
    }
  }
}
