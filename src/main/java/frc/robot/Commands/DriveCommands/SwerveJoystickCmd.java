package frc.robot.Commands.DriveCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Subsystems.Drivetrain;

public class SwerveJoystickCmd extends Command {
    private final Drivetrain drivetrain;
    private final Supplier<Double> xSpdFunction, ySpdFunction, rotFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    public SwerveJoystickCmd(Drivetrain drivetrain, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
            Supplier<Double> rotFunction, Supplier<Boolean> fieldOrientedFunction) {
        this.drivetrain = drivetrain;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.rotFunction = rotFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
        this.rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed = -ySpdFunction.get();
        double ySpeed = -xSpdFunction.get();
        double rot = rotFunction.get();

        xSpeed = Math.abs(xSpeed) < OIConstants.kDeadband ? 0 : xSpeed;
        ySpeed = Math.abs(ySpeed) < OIConstants.kDeadband ? 0 : ySpeed;
        rot = Math.abs(rot) < OIConstants.kDeadband ? 0 : rot;

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        rot = rotLimiter.calculate(rot);

        // While holding the button the robot is robot oriented, else its field oriented
        ChassisSpeeds chassisSpeeds;
       if (fieldOrientedFunction.get()) {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
       } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
            drivetrain.getRotation2d());
       }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        drivetrain.setModuleStates(moduleStates);
                                                                                                                                                     
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}