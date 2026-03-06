// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.Align;
import frc.robot.commands.swervedrive.AutoCommands.AutoShoot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import swervelib.SwerveInputStream;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  // Initialize Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opXbox = new CommandXboxController(1);

  // Swerve Initialization
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  private SendableChooser<Command> sendableChooser = new SendableChooser<>();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(.60)
      .allianceRelativeControl(true);
  
  // Initialize Subsystems
  Shooter shooter = new Shooter(); 
  Align align = new Align(swerveSubsystem);


  public RobotContainer() {

    // Configure Bindings
    configureBindings();

    // Setup optional paths
    sendableChooser.setDefaultOption("Nothing", null);
    sendableChooser.addOption("To source and back", new AutoShoot(swerveSubsystem, shooter));

    SmartDashboard.putData(sendableChooser);

    DriverStation.silenceJoystickConnectionWarning(true); // Get rid of controller error
  }

  private void configureBindings() {

    // Setup swervedrive command
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // Setup Controls


    // Driver Controller
    driverXbox.start().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    driverXbox.leftBumper().whileTrue(shooter.shootCycle()).onFalse(shooter.stopCycles());
    driverXbox.a().whileTrue(shooter.testIndxr()).onFalse(shooter.stopIndxr());
    driverXbox.b().whileTrue(align);

    // Operator Controllers
    opXbox.leftBumper().whileTrue(shooter.testShooter()).onFalse(shooter.stopShooter()); 
    //opXbox.rightBumper().whileTrue(shooter.testShooterIntake()).onFalse(shooter.stopShooterIntake());
    opXbox.a().whileTrue(shooter.testGroundIntake()).onFalse(shooter.stopGroundIntake());
    opXbox.rightBumper().whileTrue(shooter.startIntakeCycle()).onFalse(shooter.stopCycles());

  }


  public Command getAutonomousCommand() {

    try{
      return new SequentialCommandGroup( 
          new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
          sendableChooser.getSelected()

        );
    }
    catch(Exception e)
    {
      System.out.println("path did not run");
      return null;
    }

  }
  
  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
