package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class ModuleConstants {

  public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public static final double kDriveMotorGearRatio = 1.0 / 6.75;
  public static final double kTurningMotorGearRatio = 7.0 / 150.0;
  public static final double kDriveEncoderRot2Meter =
    kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  public static final double kTurningEncoderRot2Rad =
    kTurningMotorGearRatio * 2 * Math.PI;
  public static final double kDriveEncoderRPM2MeterPerSec =
    kDriveEncoderRot2Meter / 60.0;
  public static final double kTurningEncoderRPM2RadPerSec =
    kTurningEncoderRot2Rad / 60.0;
  public static final double kPTurning = .6;

  public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(
    16.5
  );
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond =
    3 * Math.PI;

  public static class ModuleSpecificConstants {

    //Absolute Encoder Offset and ID
    public double kAbsoluteEncoderOffset;
    public boolean kAbsoluteEncoderReversed;
    public Integer kAbsoluteEncoderID;

    //Module Speeds
    public double kMaxModuleSpeed;
    public double kMaxModuleAngularSpeed;

    //Encoder Direction
    public boolean kdriveEncoderReversed;
    public boolean kturningEncoderReversed;

    //Motor IDs
    public Integer kdriveMotorID;
    public Integer kturningMotorID;

    public ModuleSpecificConstants(String moduleLoc) {
      switch (moduleLoc) {
        // Front Left
        case "FL":
          kAbsoluteEncoderReversed = false;
          kAbsoluteEncoderID = 1;

          kMaxModuleSpeed = 1;
          kMaxModuleAngularSpeed = 1;

          kdriveEncoderReversed = true;
          kturningEncoderReversed = true;

          kturningMotorID = 11;
          kdriveMotorID = 12;

          break;
        // Front Right
        case "FR":
          kAbsoluteEncoderReversed = false;
          kAbsoluteEncoderID = 2;

          kMaxModuleSpeed = 1;
          kMaxModuleAngularSpeed = 1;

          kdriveEncoderReversed = false;
          kturningEncoderReversed = true;

          kturningMotorID = 21;
          kdriveMotorID = 22;
          break;
        // Back Left
        case "BL":
          kAbsoluteEncoderReversed = false;
          kAbsoluteEncoderID = 3;

          kMaxModuleSpeed = 1;
          kMaxModuleAngularSpeed = 1;

          kdriveEncoderReversed = false;
          kturningEncoderReversed = true;

          kturningMotorID = 31;
          kdriveMotorID = 32;
          break;
        // Back Right
        case "BR":
          kAbsoluteEncoderReversed = false;
          kAbsoluteEncoderID = 4;

          kMaxModuleSpeed = 1;
          kMaxModuleAngularSpeed = 1;
          kdriveEncoderReversed = true;
          kturningEncoderReversed = true;

          kturningMotorID = 41;
          kdriveMotorID = 42;
          break;
      }
    }
  }
}
