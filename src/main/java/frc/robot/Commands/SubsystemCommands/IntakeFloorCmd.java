package frc.robot.Commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Subsystems.Intake;

public class IntakeFloorCmd extends Command{
    private Intake intake;
    private Boolean isFinished = false;

    public IntakeFloorCmd(Intake intake){
        this.intake = intake;
    }

    @Override
    public void execute() {
       if (!intake.hasGamePeice){
            intake.intake();
          // intake.setIntakeAngle(IntakeConstants.kIntakeFloorAngle);
       } else {

       }
    }



    @Override
    public void end(boolean interrupted) {
        intake.stop();
       // intake.setIntakeAngle(IntakeConstants.kIntakeHome);
    }
    
}
