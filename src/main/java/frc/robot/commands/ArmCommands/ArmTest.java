package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;

public class ArmTest extends Command{
    Arm arm;
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        arm.armStop();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (arm.getArmEncoder()>12){
            arm.moveArm(0.1);
        }
        else{
            arm.moveArm(0.2);
        }
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    public ArmTest(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }
    
}
