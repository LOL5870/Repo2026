package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DashboardInserts extends SubsystemBase{

    public final Field2d gameField = new Field2d();
    public SwerveSubsystem drivebase;
    public Shooter shooter;

    public DashboardInserts(){

        SmartDashboard.putData("field", gameField);
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Flywheel Spinning", shooter.isFlywheel());
        SmartDashboard.putBoolean("Indxer Spinning", shooter.isIndxr());

    }

    @Override
    public void periodic(){
        gameField.setRobotPose(drivebase.getPose());
    }

}
