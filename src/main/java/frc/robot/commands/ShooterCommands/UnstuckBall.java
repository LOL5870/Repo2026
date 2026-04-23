package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class UnstuckBall extends Command{

    private Shooter shooter;
    private double RPM;

    public UnstuckBall(Shooter shooter, double rpm){
        this.shooter = shooter;
        this.RPM = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.begoneFuel(RPM);
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
