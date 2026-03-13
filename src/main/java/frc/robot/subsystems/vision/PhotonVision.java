package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase{

    private final PhotonCamera camera; 
    public PhotonPipelineResult results;

    public PhotonVision(String camName){

        camera = new PhotonCamera(camName);

        SmartDashboard.putBoolean("checkHopper", results.hasTargets());
        SmartDashboard.putData("checkHopper test", checkHopper());
    }

    public Command checkHopper(){
        return run(() -> {
            camera.getLatestResult();
            results.hasTargets();
        });
    }

    public boolean getResults(){
        return results.hasTargets();
    }
}
