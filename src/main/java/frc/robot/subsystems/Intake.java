package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    private SparkMax motor = new SparkMax(0, MotorType.kBrushless);

    public void IntakeMotorForwards(double speed){
        motor.set(speed);
    }

    public void IntakeMotorBackwards(double speed){
        motor.set(-speed);
    }

    public void IntakeStop(){
        motor.stopMotor();

    }


}
