package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmStop extends Command{

Arm arm;

    @Override
    public void end(boolean interrupted) {
        arm.armStop();
    }

    @Override
    public void execute() {
        if (arm.getArmEncoder()>20){
            arm.armStop();
        } else{
            arm.moveArm(0.1);
        }
    }

 
    public ArmStop(Arm arm){
this.arm=arm;
addRequirements(arm);
    }
    
}
