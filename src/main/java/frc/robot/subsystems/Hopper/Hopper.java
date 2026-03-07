package frc.robot.subsystems.Hopper;

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

    SparkMax rightMotor;
    SparkMax leftMotor;

    SparkMaxConfig leaderConfig;
    SparkMaxConfig followConfig;
    SparkMaxConfig globaldConfig;

    public Hopper(){

        rightMotor =  new SparkMax(HopperConstants.rightMotorID, MotorType.kBrushless);
        leftMotor =  new SparkMax(HopperConstants.leftMotorID, MotorType.kBrushless);

        leaderConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();

        globaldConfig.idleMode(IdleMode.kBrake);
        leaderConfig.apply(globaldConfig);
        followConfig.apply(globaldConfig).follow(rightMotor, true);

        rightMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public Command hopperIn(){
        return run(()-> rightMotor.set(0.2));
    }

    public Command hopperOut(){
        return run(()-> rightMotor.set(-0.2));
    }
    
    public Command stopHopper(){
        return run((()-> rightMotor.stopMotor()));
    }
}
