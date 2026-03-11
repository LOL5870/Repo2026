package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShoot extends Command{

    private Supplier<Double> shooterRPM;
    private Shooter shooter;
    private boolean rpmReached; 

    public AutoShoot(Supplier<Double> shooterRPM, Shooter shooter){
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
        shooter.setRPM(shooterRPM);

        if(shooter.getShooterRPM() > shooterRPM.get() - 300 && !rpmReached) { 
            rpmReached = true;
        }

        if(rpmReached)
            shooter.feedFuel();
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
    
}
