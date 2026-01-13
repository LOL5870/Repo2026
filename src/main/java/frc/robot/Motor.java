package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    SparkMax motor = new SparkMax(15, MotorType.kBrushless);


    public Command move1() {
        return new InstantCommand(() -> {
            motor.set(1);
        });
    }
    public Command stop1() {
        return new InstantCommand(() -> {
            motor.set(0);
        });
    }
}
