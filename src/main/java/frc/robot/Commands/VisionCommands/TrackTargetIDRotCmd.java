package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class TrackTargetIDRotCmd extends Command {

    private Drivetrain drivetrain;
    private Integer targetID = 0;

    public TrackTargetIDRotCmd(Drivetrain drivetrain, Integer targetID) {
        this.drivetrain = drivetrain;
        this.targetID = targetID;
    }

    @Override
    public void execute() {
        drivetrain.trackTargetIDRotation(targetID);
    }
    
}
