package frc.robot.subsystems.shooter;

import java.util.function.Supplier;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    SparkMax shooter1 = new SparkMax(10, MotorType.kBrushless); 
    SparkMax shooterIntake = new SparkMax(11, MotorType.kBrushless); 
    SparkMax indxr = new SparkMax(12, MotorType.kBrushless); 
    SparkMax grndIntake = new SparkMax(13, MotorType.kBrushless); 

    public Command shoot(Supplier<Double> spd) { 
        return run(() -> {
            shooterIntake.set(-.6);
        }); 
    }

        public Command shootReverse(Supplier<Double> spd) { 
        return run(() -> {
            indxr.set(spd.get());
        }); 
    }

    // Sometimes my genius its... 
        public Command IntakeToHopper() { 
        return run(() -> {
            shooterIntake.set(-0.7);
            indxr.set(-0.8);
            grndIntake.set(0.7);
        });
    }
    // its terifying
        public Command IntakeToShooter() { 
        return run(() -> {
            shooterIntake.set(-0.7);
            indxr.set(0.8);
            grndIntake.set(0.7);
        }); 
    }

    public Command stop() { 
        return run(() -> {
            shooter1.set(0);
            shooterIntake.set(0);
            indxr.set(0);
            grndIntake.set(0);
            grndIntake.set(0);
        }); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FlyWheel", shooterIntake.get());
    }
}