package frc.robot.commands.swervedrive.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShoot extends Command{

    private Supplier<Double> shooterIntakeRPM;
    private Supplier<Double> shooterRPM;
    private Shooter shooter;
    private boolean rpmReached;

    public AutoShoot(Supplier<Double> shooterRPM,Supplier<Double> shooterIntakeRPM, Shooter shooter){

        this.shooterRPM = shooterRPM;
        this.shooterIntakeRPM = shooterIntakeRPM;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rpmReached = false;
    }

    @Override
    public void execute() {

        shooter.setIntakeRPM(shooterIntakeRPM);
        shooter.setShooterRPM(shooterRPM);

        if(shooter.getShooterRPM() > shooterRPM.get() - 500  && shooter.getShooterIntakeRPM() > shooterIntakeRPM.get() - 500 && !rpmReached){
            rpmReached = true;
        }

        if(rpmReached){
            shooter.feedFuel();
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeed();
        shooter.stopIndxr();
        shooter.stopShooter();
        shooter.stopShooterIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    
    
}
