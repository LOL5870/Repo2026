package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
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


public class HubAlign extends Command{
    
    private SwerveSubsystem swerveSubsystem; 
    private PIDController rotController; 
    private double xTarget;
    private CommandXboxController controller;
    private int[] tagIDs;
    private Supplier<Boolean> isOscillate; 
    private double xSpeed; 
    private boolean isSet; 
    

    public HubAlign(SwerveSubsystem subsystem, CommandXboxController controller, Supplier<Boolean> isOscillate) {

        this.swerveSubsystem = subsystem;
        this.controller = controller;
        this.isOscillate = isOscillate; 
        rotController = new PIDController(AutoConstants.HUB_ALIGN_PID.kP, AutoConstants.HUB_ALIGN_PID.kI, AutoConstants.HUB_ALIGN_PID.kD);
        addRequirements(swerveSubsystem);
    }


    @Override
    public void initialize() {
        if(DriverStation.getAlliance().get() == Alliance.Red)
            tagIDs = AprilTagIDs.RED_HUB_APRIL_TAGS;
        else
            tagIDs = AprilTagIDs.BLUE_HUB_APRIL_TAGS;

        xSpeed = 0; 
    }

    @Override
    public void execute() {

        // Middle
        if(findID(tagIDs[TAGS.middle.value]) && !findID(tagIDs[TAGS.left.value]) && !findID(tagIDs[TAGS.right.value])){
            xTarget = 0;
        }

        double rot = rotController.calculate(LimelightHelpers.getTX("limelight"), xTarget);

        if(isOscillate.get() && !isSet) { 
            xSpeed = 0.5;
            isSet = true; 
        }
        else { 
            xSpeed = -controller.getLeftX(); 
            isSet = false; 
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-controller.getLeftY(), xSpeed, rot, swerveSubsystem.getHeading());
        swerveSubsystem.setChassisSpeeds(chassisSpeeds);

        xSpeed *= -1; 
    }

    @Override
    public void end(boolean interrupted) {
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