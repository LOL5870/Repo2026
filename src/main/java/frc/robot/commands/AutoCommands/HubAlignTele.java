package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TAGS;
import frc.robot.Constants.AprilTagIDs;

// PID wiggle test

public class HubAlignTele extends Command{
    
    private SwerveSubsystem swerveSubsystem; 
    private PIDController rotController; 
    private double xTarget;
    private CommandXboxController controller;
    private int[] tagIDs;
    

    public HubAlignTele(SwerveSubsystem subsystem, CommandXboxController controller) {

        this.swerveSubsystem = subsystem;
        this.controller = controller;

        rotController = new PIDController(AutoConstants.HUB_ALIGN_PID_TELE.kP, AutoConstants.HUB_ALIGN_PID_TELE.kI, AutoConstants.HUB_ALIGN_PID_TELE.kD);
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

        double rot = rotController.calculate(LimelightHelpers.getTX("limelight"), xTarget);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-controller.getLeftY(), -controller.getLeftX(), rot, swerveSubsystem.getHeading());
        swerveSubsystem.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Check for valid id
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