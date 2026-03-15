package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagIDs;
import frc.robot.Constants.TAGS;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class ShootCycle extends Command{

    private Supplier<Double> shooterRPM;
    private Shooter shooter;
    private Supplier<Boolean> isFeed;

    public ShootCycle(Supplier<Double> shooterRPM, Shooter shooter, Supplier<Boolean> isFeed){
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
