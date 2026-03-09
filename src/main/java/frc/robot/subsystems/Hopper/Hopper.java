package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Hopper extends SubsystemBase{

    // Define motors
    SparkMax leaderMotor;
    SparkMax followMotor;

    // Define configs
    SparkMaxConfig leaderConfig;
    SparkMaxConfig followConfig;
    SparkMaxConfig globaldConfig;

    public Hopper(){

        // Initalize motors   --Change this in the Constants.java file--
        leaderMotor =  new SparkMax(HopperConstants.rightMotorID, MotorType.kBrushless);
        followMotor =  new SparkMax(HopperConstants.leftMotorID, MotorType.kBrushless);

        // Initalize motor configs
        leaderConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();
        globaldConfig.idleMode(IdleMode.kBrake);

        // Initalize leader and follower config
        leaderConfig.apply(globaldConfig);
        followConfig.apply(globaldConfig).follow(leaderMotor, true);

        // Initialize leader and follower motor config
        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public Command hopperIn(){
        return run(()-> leaderMotor.set(0.2));
    }

    public Command hopperOut(){
        return run(()-> leaderMotor.set(-0.2));
    }
    
    public Command stopHopper(){
        return run((()-> leaderMotor.stopMotor()));
    }
}
