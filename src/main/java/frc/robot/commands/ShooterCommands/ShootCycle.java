package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCycle extends Command{

    private Supplier<Double> shooterRPM;
    private Shooter shooter;
    private Supplier<Boolean> isFeed;
    private boolean rpmReached;

    public ShootCycle(Supplier<Double> shooterRPM, Shooter shooter, Supplier<Boolean> isFeed){
        this.isFeed = isFeed;
        this.shooterRPM = shooterRPM;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rpmReached = false;
    }

    @Override
    public void execute() {
        shooter.setRPM(shooterRPM.get());

        if(-shooter.getShooterRPM() > shooterRPM.get() - 1000 && !rpmReached) { 
            rpmReached = true;
            Constants.rpmReached = true;
        }

        if(isFeed.get() && rpmReached){
            shooter.feedFuel();
        }

        else{
            shooter.stopFeed();
            rpmReached = false;
            Constants.rpmReached = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopCycles();
        Constants.rpmReached = false; 
        rpmReached = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
