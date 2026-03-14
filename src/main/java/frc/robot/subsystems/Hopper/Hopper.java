package frc.robot.subsystems.hopper;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase{

    // Define motors
    VictorSP leaderMotor;
    VictorSP followMotor;

    public Hopper(){

        // Initalize motors   --Change this in the Constants.java file--
        leaderMotor =  new VictorSP(HopperConstants.leftMotorID);
        followMotor =  new VictorSP(HopperConstants.rightMotorID);

        SmartDashboard.setDefaultNumber("Hopper Speed", 0); 
        SmartDashboard.setDefaultNumber("Hopper Timeout", 0); 
    }


    public Command hopperOut(Supplier<Double> speed){
        System.out.println(speed.get());
        return run(()-> {
            leaderMotor.set(speed.get());
            followMotor.set(speed.get()*1.1);

        });
    }

    public Command spinLeftMotor(Supplier<Double> speed) { 
        return run(()-> {
            leaderMotor.set(speed.get()*1.2);
        });
    }

    public Command spinRightMotor(Supplier<Double> speed) { 
        return run(()-> {
            followMotor.set(speed.get());
        });
    }

    public Command hopperIn(Supplier<Double> speed){
        return run(()-> {
            leaderMotor.set(-speed.get());
            followMotor.set(-speed.get()*1.2);

        });
    }

    public Command stopLeftMotor() { 
        return runOnce(() -> leaderMotor.stopMotor()); 
    }

    public Command stopRightMotor() { 
        return runOnce(() -> followMotor.stopMotor()); 
    }

    public Command oscillateHopper() { 
        System.out.println("HOPPER CYCLE STARTING");
        return new SequentialCommandGroup(
            hopperIn(() -> 0.75).withTimeout(.1),
            hopperOut(() -> 0.75).withTimeout(.1)
        );
    }

    public Command oscillationPrep(){
        return hopperIn(() -> 0.4).withTimeout(.75); 
    }
    
    public Command hopperExtend(){
        return hopperOut(() -> 0.4).withTimeout(1.75); 
    }

    public Command stopHopper(){
        return run(()-> {
            leaderMotor.stopMotor();
            followMotor.stopMotor();;
        });    }


        
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Hopper Speed", leaderMotor.get());
    }
}
