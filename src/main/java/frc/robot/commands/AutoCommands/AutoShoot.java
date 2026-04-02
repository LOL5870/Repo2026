package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagIDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class AutoShoot extends Command{

    private Supplier<Double> shooterRPM;
    private Shooter shooter;
    private boolean rpmReached; 
    private int[] tagIDs; 

    public AutoShoot(Supplier<Double> shooterRPM, Shooter shooter){
        this.shooterRPM = shooterRPM;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
         if(DriverStation.getAlliance().get() == Alliance.Red)
            tagIDs = AprilTagIDs.RED_HUB_APRIL_TAGS;
        else
            tagIDs = AprilTagIDs.BLUE_HUB_APRIL_TAGS;
        rpmReached = false; 
    }

    @Override
    public void execute() {

        if(findID(tagIDs[Constants.TAGS.left.value]) || findID(tagIDs[Constants.TAGS.right.value]) || findID(tagIDs[Constants.TAGS.middle.value])){
                shooter.setRPM(shooterRPM.get());  

            if(-shooter.getShooterRPM() > shooterRPM.get() - 1000 && !rpmReached) { 
                rpmReached = true;
                
            }

            if(rpmReached)
                shooter.feedFuel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.stopIndxr();
        shooter.stopIntakeFlaps();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    private boolean findID(double id) { 
        RawFiducial idList[] = LimelightHelpers.getRawFiducials("limelight"); 
        
        for(int i = 0; i < idList.length; i++){
                
            if(id == idList[i].id){
                return true;
            }            
        }
        
        return false;
    }
}
