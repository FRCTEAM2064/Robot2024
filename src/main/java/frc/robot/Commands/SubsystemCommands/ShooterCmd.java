package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Shooter;

public class ShooterCmd extends InstantCommand{
    private Shooter shooter;
    private Integer nmber = 0;
    public ShooterCmd(Shooter shooter){
        this.shooter = shooter;

    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("num of shots", nmber);
        shooter.shoot();
    }

}
