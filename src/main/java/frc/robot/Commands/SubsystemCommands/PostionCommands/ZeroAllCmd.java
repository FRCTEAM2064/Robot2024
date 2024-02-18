package frc.robot.Commands.SubsystemCommands.PostionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Wrist;

public class ZeroAllCmd extends InstantCommand{
    private Elevator elevator;
    private Wrist wrist;
    private Intake intake;
    
    public ZeroAllCmd(Elevator elevator, Wrist wrist, Intake intake){
        this.elevator = elevator;
        this.wrist = wrist;
        this.intake = intake;
    }

    @Override
    public void execute() {
        elevator.zeroElevator();
        intake.zeroIntake();
        wrist.zeroWrist();
     }
}
