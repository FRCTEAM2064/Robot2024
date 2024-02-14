package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist;

public class WristCmd extends Command{
    private Wrist wrist;

    public WristCmd(Wrist wrist){
        this.wrist = wrist;
    }
    
    @Override
    public void execute(){
        wrist.setWrist(1);
    }

}
