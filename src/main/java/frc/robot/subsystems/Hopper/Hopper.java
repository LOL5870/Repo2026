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
    VictorSP leftMotor;
    VictorSP rightMotor;
    private final double leftMultiplier = 1.175; 
    public Hopper(){

        // Initalize motors   --Change this in the Constants.java file--
        leftMotor =  new VictorSP(HopperConstants.leftMotorID);
        rightMotor =  new VictorSP(HopperConstants.rightMotorID);

        SmartDashboard.setDefaultNumber("Left Hopper Speed", 0); 
        SmartDashboard.setDefaultNumber("Right Hopper Speed", 0); 
    }


    public Command hopperOut(Supplier<Double> speed){
        System.out.println(speed.get());
        return run(()-> {
            leftMotor.set(speed.get() * leftMultiplier);
            rightMotor.set(speed.get());

        });
    }

    public Command hopperIn(Supplier<Double> speed){
        return run(()-> {
            leftMotor.set(-speed.get() * leftMultiplier);
            rightMotor.set(-speed.get());

        });
    }

    public Command stopLeftMotor() { 
        return runOnce(() -> leftMotor.stopMotor()); 
    }

    public Command stopRightMotor() { 
        return runOnce(() -> rightMotor.stopMotor()); 
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

    public Command extendHopperCustom(){
        return run(()->{
            leftMotor.set(SmartDashboard.getNumber("Left Hopper Speed", 0));
            rightMotor.set(SmartDashboard.getNumber("Right Hopper Speed", 0));

        });
    }

    public Command stopHopper(){
        return run(()-> {
            leftMotor.stopMotor();
            rightMotor.stopMotor();;
        });    }

}
