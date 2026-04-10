package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCycleMan extends Command{

    private Shooter shooter;
    private double RPM;
    private Supplier<Boolean> isFeed;
    private Supplier<Boolean> isHopper;
    private boolean rpmReached;

    public ShooterCycleMan(Shooter shooter, double rpm, Supplier<Boolean> isFeed, Supplier<Boolean> isHopper){

        this.isHopper = isHopper;
        this.isFeed = isFeed;
        this.shooter = shooter;
        this.RPM = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rpmReached = false;
    }

    @Override
    public void execute() {

        shooter.setRPM(RPM);

        if(-shooter.getShooterRPM() > RPM - 1000 && !rpmReached) { 
            rpmReached = true;
            
        }


        if(isFeed.get()){
            shooter.feedFuel();
        }

        else if(isHopper.get()){
            shooter.fromGroundFeed();
        }

        else{
            shooter.stopFromGround();
            shooter.stopFeed();
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopCycles();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}