package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCycle extends Command{

    private Supplier<Double> shooterRPM;
    private Supplier<Boolean> isFeed;
    private Shooter shooter;

    public ShootCycle(Supplier<Double> shooterRPM, Supplier<Boolean> isFeed, Shooter shooter){
        this.isFeed = isFeed;
        this.shooterRPM = shooterRPM;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setRPM(shooterRPM.get());
        if(isFeed.get()){
            shooter.feedFuel();
        }
        else{
            shooter.stopFeed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.stopIndxr();
        shooter.stopIntakeFlaps();
        shooter.stopCycles();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
