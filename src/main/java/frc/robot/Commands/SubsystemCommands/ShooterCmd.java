package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ShooterCmd extends Command{
    private Shooter shooter;
    public ShooterCmd(Shooter shooter){
        this.shooter = shooter;
    }
    @Override
    public void execute(){
        shooter.runAll();
    }
}
