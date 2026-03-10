package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
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

        followMotor.setInverted(true);
        leaderMotor.addFollower(followMotor);
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
