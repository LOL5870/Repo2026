package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeForward extends Command{

    private Intake Intake;

    public IntakeForward(Intake Intake){

        this.Intake = Intake;
        addRequirements(Intake);

    }

    public void execute(){
        Intake.IntakeMotorForwards(5);

    }

    public void end(boolean interrupted){
        Intake.IntakeStop();

    }

    public boolean isFinished(){
        return true;
    }

}
