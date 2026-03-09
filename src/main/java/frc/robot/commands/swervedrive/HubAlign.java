package frc.robot.commands.swervedrive;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TAGS;
import frc.robot.Constants.AprilTagIDs;


public class HubAlign extends Command{
    
    private Supplier<Double> controllerX, controllerY; 
    private SwerveSubsystem swerveSubsystem; 
    private PIDController rotController; 
    private double xTarget; 
    private int[] tagIDs; 
    
    
    public HubAlign(Supplier<Double> controllerX, Supplier<Double> controllerY, SwerveSubsystem subsystem) {

        this.controllerX = controllerX;
        this.controllerY = controllerX;
        this.swerveSubsystem = subsystem;

        rotController = new PIDController(AutoConstants.HUB_ALIGN_PID.kP, AutoConstants.HUB_ALIGN_PID.kI, AutoConstants.HUB_ALIGN_PID.kD);
        addRequirements(swerveSubsystem);
    }


    @Override
    public void initialize() {
        if(DriverStation.getAlliance().get() == Alliance.Red)
            tagIDs = AprilTagIDs.RED_HUB_APRIL_TAGS; 
        else
            tagIDs = AprilTagIDs.BLUE_HUB_APRIL_TAGS;
    }

    @Override
    public void execute() {

        // Middle
        if(findID(tagIDs[TAGS.middle.value]) && !findID(tagIDs[TAGS.left.value]) && !findID(tagIDs[TAGS.right.value])){
            xTarget = 0;
        }
        // Left
        else if(!findID(tagIDs[TAGS.middle.value]) && findID(tagIDs[TAGS.left.value]) && !findID(tagIDs[TAGS.right.value])){
            xTarget = AutoConstants.xOffsetSide;
        }
        // Right
        else if(!findID(tagIDs[TAGS.middle.value]) && !findID(tagIDs[TAGS.left.value]) && findID(tagIDs[TAGS.right.value])){
            xTarget = -AutoConstants.xOffsetSide;
        }
        // Middle-Left
        else if(findID(tagIDs[TAGS.middle.value]) && findID(tagIDs[TAGS.left.value]) && !findID(tagIDs[TAGS.right.value])){
            xTarget = AutoConstants.xOffsetCorner;
        }
        // Middle-right
        else if(findID(tagIDs[TAGS.middle.value]) && !findID(tagIDs[TAGS.left.value]) && findID(tagIDs[TAGS.right.value])){
            xTarget = -AutoConstants.xOffsetCorner;
        }

        double rot = rotController.calculate(LimelightHelpers.getTX("limelight"), xTarget);
        // double rot = 0; 

        swerveSubsystem.drive(new Translation2d(controllerX.get(), controllerY.get()), rot, true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean findID(double id) { 
        RawFiducial idList[] = LimelightHelpers.getRawFiducials("limelight"); 
        
        for(int i = 0; i < idList.length; i++){
                
            if(id == idList[i].id){
                return true;
            }            
        }
        
        return false;
    }
    
    
    
    
}