package frc.robot.commands.swervedrive.ShooterCommands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCycle extends Command{

    private Supplier<Double> shooterIntakeDist, shooterDist;
    private Supplier<Boolean> isIndxr;
    private Shooter shooter;

    public ShootCycle(Supplier<Double> shooterIntakeDist, Supplier<Double> shooterDist, Supplier<Boolean> isIndxr, Shooter shooter){


        this.isIndxr = isIndxr;
        this.shooterDist = shooterDist;
        this.shooterIntakeDist = shooterIntakeDist;
        this.shooter = shooter;
        
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.shooterIntakeController.setSetpoint(-shooterIntakeDist.get(), ControlType.kMAXMotionVelocityControl); 
        shooter.shooterController.setSetpoint(shooterDist.get(), ControlType.kMAXMotionVelocityControl);

        if(isIndxr.get()){
            shooter.startIndxr();
        }
        else{
            shooter.indxr.stopMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.stopShooterIntake();
        shooter.stopIndxr();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
