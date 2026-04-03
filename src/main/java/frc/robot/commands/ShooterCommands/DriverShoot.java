package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.AutoShoot;
import frc.robot.subsystems.shooter.Shooter;

public class DriverShoot extends Command{

    private Supplier<Double> shooterRPM;
    private Shooter shooter;
    private boolean rpmReached;
    private AutoShoot autoShoot;
    private int[] tagIDs;

    public DriverShoot(Supplier<Double> shooterRPM, Shooter shooter, AutoShoot autoShoot){
        this.shooterRPM = shooterRPM;
        this.shooter = shooter;
        this.autoShoot = autoShoot;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rpmReached = false;
    }

    @Override
    public void execute() {

        if(autoShoot.findID(tagIDs[Constants.TAGS.left.value])){
            shooter.setRPM(shooterRPM.get());
        }

        if(-shooter.getShooterRPM() > shooterRPM.get() - 1000 && !rpmReached) { 
            rpmReached = true;
            
        }

        if(rpmReached){
            shooter.feedFuel();
        }

        else{
            shooter.stopFeed();
            shooter.stopShooter().withTimeout(0.5);
            rpmReached = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopCycles();
        rpmReached = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}