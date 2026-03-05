// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import java.io.File;

public class RobotContainer {

  // Initialize Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opXbox = new CommandXboxController(1);

  // Initialize Subsystems
  Shooter shooter = new Shooter(); 

  // Swerve Initialization
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(.60)
      .allianceRelativeControl(true);

  public RobotContainer() {
    // Default Commands
    //
    //

    // Configure Bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true); // Get rid of controller error
  }

  private void configureBindings() {

    // Setup swervedrive command
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // Setup Controls


    // Driver Controller
    driverXbox.start().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    driverXbox.rightBumper().whileTrue(shooter.testShooter()).onFalse(shooter.stopShooter()); 
    driverXbox.leftBumper().whileTrue(shooter.testShooterIntake()).onFalse(shooter.stopShooterIntake()); 
    driverXbox.a().whileTrue(shooter.testGroundIntake()).onFalse(shooter.stopGroundIntake()); 
    driverXbox.y().whileTrue(shooter.shootCycle()).onFalse(shooter.stopCycles());

    // Operator Controllers
    opXbox.b().whileTrue(shooter.startIntakeCycle()).onFalse(shooter.stopCycles());
    opXbox.a().whileTrue(shooter.ejectFuel()).onFalse(shooter.stopCycles());

  }


  public Command getAutonomousCommand() {
    return null; 
  }
  
  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
