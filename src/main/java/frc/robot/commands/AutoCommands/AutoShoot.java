package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.PhotonVision;

public class AutoShoot extends Command{

    private Supplier<Double> shooterRPM;
    private Shooter shooter;
    private PhotonVision vision;
    private boolean rpmReached;
    private boolean hopperHas;
    
    public AutoShoot(Supplier<Double> shooterRPM, Shooter shooter, PhotonVision vision){
        this.shooterRPM = shooterRPM;
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter, vision);
    }

    @Override
    public void initialize() {
        rpmReached = false;
        hopperHas = false;
    }

    @Override
    public void execute() {
        shooter.setRPM(shooterRPM.get());

        if(vision.getResults() == true){
            hopperHas = true;
        }

        SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());
        if(!hopperHas){
            if(-shooter.getShooterRPM() > shooterRPM.get() - 300 && !rpmReached) { 
                System.out.println("ASKDJLASJD");
                rpmReached = true;
            }
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
