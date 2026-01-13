package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmBounce extends Command{
    Arm arm;
    double speed;
    
int counter=0;
    //boolean direction; //true is ccw (positive)
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        arm.armStop();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stu
        while (counter<5){
        arm.moveArm(speed);
        if (arm.getArmEncoder()>25){//if arm is up,go cw
            speed=-0.2;
            counter+=1;
          System.out.println(counter);

        }
        if (arm.getArmEncoder()<0){ //if arm is at bottom position, go ccw
            speed=0.2;
          //  System.out.println("ITS CCW");
        }
    
    } end(true);
    }

    public ArmBounce(Arm arm){
        this.arm=arm;
        this.speed=0.2;
        addRequirements(arm);
    }
    
}
