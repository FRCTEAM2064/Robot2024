package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class ResetHeadingCmd extends Command {
    private final Drivetrain drivetrain;

    public ResetHeadingCmd(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        drivetrain.zeroHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
