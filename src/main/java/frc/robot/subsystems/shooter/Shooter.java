package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterIntakeConstants;
import frc.robot.subsystems.hopper.Hopper;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {
    
    // Define Motors
    public SparkMax shooterLeft; 
    public SparkMax shooterRight;    

    public Shooter(){
        
        // Initialize the Motors
        shooterLeft = new SparkMax(ShooterIntakeConstants.ShooterLeftID, MotorType.kBrushless); 
        shooterRight = new SparkMax(ShooterIntakeConstants.ShooterRightID, MotorType.kBrushless); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterLeft.get());
        SmartDashboard.putNumber("Shooter Speed", shooterRight.get());
    }   

    public Command spinLeftMotor(double speed) { 
        return run(() -> shooterLeft.set(speed)); 
    }

}