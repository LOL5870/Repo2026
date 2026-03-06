package frc.robot.commands.swervedrive.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.Align;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoShoot extends SequentialCommandGroup{

    public AutoShoot(SwerveSubsystem swerveSubsystem, Shooter shooter){
        
        new AutoBuilder();

        addCommands(
            AutoBuilder.buildAuto("path 1"),
            new WaitCommand(5),
            new Align(swerveSubsystem),
            new WaitCommand(1), 
            new Shooter().shootCycle(),
            new WaitCommand(5),
            new Shooter().stopCycles()
        );            
    }
}
