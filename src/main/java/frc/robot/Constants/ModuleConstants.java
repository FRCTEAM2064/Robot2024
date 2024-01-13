package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class ModuleConstants {

public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
public static final double kDriveMotorGearRatio = 1.0 / 6.75;
public static final double kTurningMotorGearRatio = 7.0 / 150.0;
public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
public static final double kPTurning = .6;

public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(16.5);
public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

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

    public ModuleSpecificConstants(Integer moduleID, String moduleLoc)
    {
        switch (moduleID) {
            /* **********Mentor BOT************* */
            // Module 1
            case 1:
                kAbsoluteEncoderOffset = 0.17724609375;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 1;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 2
            case 2:
                kAbsoluteEncoderOffset = -0.886474609375;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 2;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 3
            case 3:                
                kAbsoluteEncoderOffset = -0.18505859375;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 3;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 4
            case 4:
                kAbsoluteEncoderOffset = -0.7998046875;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 4;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            /* **********School BOT************* */
            // Module 5
            // Front Left
            case 5:
            // Goodish value: .122
                kAbsoluteEncoderOffset = 0.124;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 5;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 6
            // Rear Left
            case 6:
                //-0.55
                kAbsoluteEncoderOffset = -0.040;                
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 6;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

            // Module 7
            // Rear Right
            case 7:
            //.039
                // kAbsoluteEncoderOffset = -.049;
                kAbsoluteEncoderOffset = 0.050;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 7;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;
            
            // Module 8
            // Front Right
            case 8:
                kAbsoluteEncoderOffset = 0.573;
                kAbsoluteEncoderReversed = false;
                kAbsoluteEncoderID = 8;

                kMaxModuleSpeed = 1;
                kMaxModuleAngularSpeed = 1;
                break;

        }

        switch (moduleLoc) {
            // Front Left
            case "FL":
                // Other Bot
                //kdriveEncoderReversed = true;
                //kturningEncoderReversed = true;

                //kdriveMotorID = 11;
                //kturningMotorID = 12;
                //break;


                //School Bot
                 kdriveEncoderReversed = true;
                 kturningEncoderReversed = true;

                 kdriveMotorID = 18;
                 kturningMotorID = 17;
                 break;
            
            // Front Right
            case "FR":
                //Other Bot
                //kdriveEncoderReversed = false;
                //kturningEncoderReversed = true;

                //kdriveMotorID = 17;
                //kturningMotorID = 18;
                //break;

                //School Bot
                 kdriveEncoderReversed = false;
                 kturningEncoderReversed = true;

                 kdriveMotorID = 12;
                 kturningMotorID = 11;
                 break;

            // Back Left
            case "BL":
                //Other Bot
                //kdriveEncoderReversed = true;
                //kturningEncoderReversed = true;

                //kdriveMotorID = 13;
                //kturningMotorID = 14;
                //break;
            
                //School Bot
                 kdriveEncoderReversed = false;
                 kturningEncoderReversed = true;

                 kdriveMotorID = 16;
                 kturningMotorID = 15;
                 break;

            // Back Right
            case "BR":

                //Other Bot
                //kdriveEncoderReversed = false;
                //kturningEncoderReversed = true;

                //kdriveMotorID = 15;
                //kturningMotorID = 16;
                //break;

                //School Bot
                 kdriveEncoderReversed = true;
                 kturningEncoderReversed = true;

                 kdriveMotorID = 14;
                 kturningMotorID = 13;
                 break;
            
        }
}
}
}
