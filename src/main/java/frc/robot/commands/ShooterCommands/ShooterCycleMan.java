package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCycleMan extends Command{

    private boolean rpmReached;
    private Shooter shooter;
    private double RPM;

    public ShooterCycleMan(Shooter shooter, double rpm){

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

        if(-shooter.getShooterRPM() > RPM - 300 && !rpmReached){
            rpmReached = true;
        }

        if(rpmReached){
            shooter.feedFuel();
        }

        else{
            shooter.stopFeed();
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
