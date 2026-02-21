package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command{

    private Shooter Shooter;
    private Supplier<Double> rightTrigger;

    public Shoot(Shooter Shooter, Supplier<Double> supplier){
        this.Shooter = Shooter;
        addRequirements(Shooter);
    }

    public void execute(){
        double speed = rightTrigger.get();
        Shooter.ShooterMotorForwards(speed);

    }

    public void end(boolean interrupted){
        Shooter.ShooterStop();

    }

    public boolean isFinished(){
        return true;
    }
    
}
