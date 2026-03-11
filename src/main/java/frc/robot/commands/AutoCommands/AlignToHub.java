// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToHub extends Command {
    private PIDController yController;

    private SwerveSubsystem drivebase;
    private Supplier<Double> ysetpoint;

    public AlignToHub(SwerveSubsystem drivebase, Supplier<Double> ysetpoint) {
        
        yController = new PIDController(1.5, 0.0, 0); // Horitontal movement
        this.drivebase = drivebase;
        this.ysetpoint = ysetpoint;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {

        yController.setTolerance(.2); // horizontal tolerance
        LimelightHelpers.setPriorityTagID("limelight", 0); // only run if it sees this command
    }

    @Override
    public void execute() {

        // if camera has detected a april tag...
        if (LimelightHelpers.getTV("limelight")) {

            double ySpeed = -yController.calculate(LimelightHelpers.getTA("limelight"), ysetpoint.get());

            drivebase.drive(new Translation2d(ySpeed, 0), 0, false);

            System.out.println(LimelightHelpers.getTA("limelight"));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(), 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    }