// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Align extends Command {
    private PIDController xController, yController, rotController;

    private Timer dontSeeTagTimer, stopTimer;
    private SwerveSubsystem drivebase;
    private double tagID = -1;

    public Align(SwerveSubsystem drivebase) {
        
        xController = new PIDController(0.05, 0.0, 0); // Distance movement
        yController = new PIDController(1.5, 0.0, 0); // Horitontal movement
        rotController = new PIDController(0.05, 0, 0); // Rotation
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(0); // rotation value end point
        rotController.setTolerance(1); // rotation tolerance

        xController.setSetpoint(0); // distance value end point
        xController.setTolerance(0.9); // distance tolerance

        yController.setSetpoint(1.5); // horizontal value end point
        yController.setTolerance(1.7); // horizontal tolerance

        tagID = LimelightHelpers.getFiducialID("limelight"); // getting the ID from the april tag
    }

    @Override
    public void execute() {
        // if camera has detected a april tag...
        if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");

            double xSpeed = xController.calculate(LimelightHelpers.getTX("limelight"));
            double ySpeed = -yController.calculate(LimelightHelpers.getTA("limelight"));
            double rotValue = -rotController.calculate(postions[4]);

            drivebase.drive(new Translation2d(ySpeed, -xSpeed), rotValue, false);

            // if the robot is not at the end point
            if (!rotController.atSetpoint() ||
                    !yController.atSetpoint() ||
                    !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            drivebase.drive(new Translation2d(), 0, false);
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(), 0, false);
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long
        // as it gets a tag in the camera
        return false;
    }
}