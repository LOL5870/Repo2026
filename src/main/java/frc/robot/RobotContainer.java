// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands.Align;
import frc.robot.commands.AutoCommands.AutoShoot;
import frc.robot.commands.AutoCommands.HubAlign;
import frc.robot.commands.ShooterCommands.ShootCycle;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import swervelib.SwerveInputStream;
import java.io.File;

import javax.naming.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

public class RobotContainer {

  // Initialize Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opXbox = new CommandXboxController(1);
  InterpolatingDoubleTreeMap  shooterTreeMap = new InterpolatingDoubleTreeMap();
   
  // Swerve Initialization
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(0.1)
      .scaleTranslation(.60)
      .allianceRelativeControl(true);
  
  // Initialize Subsystems
    Shooter shooter = new Shooter(); 
    Align align = new Align(swerveSubsystem);
    Hopper hopper = new Hopper();
  

  public RobotContainer() {

    // Shooter tree map
    shooterTreeMap.put(0.965, 3475.0); 
    shooterTreeMap.put(0.765, 3525.0); 
    shooterTreeMap.put(0.545, 3675.0);
    shooterTreeMap.put(0.375, 3825.0); 
    shooterTreeMap.put(0.315, 3850.0); 
    shooterTreeMap.put(0.235, 4000.0); 

    NamedCommands.registerCommand("shootCycleMiddle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(5));
    NamedCommands.registerCommand("shootCycle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(7));
    NamedCommands.registerCommand("hubAlign", new HubAlign(swerveSubsystem, driverXbox));
    NamedCommands.registerCommand("stopCycles", shooter.stopCycles());
    NamedCommands.registerCommand("runIntake", shooter.startIntakeCycle());
    NamedCommands.registerCommand("oscillateHopper", hopper.oscillateHopper()
     );
    //NamedCommands.registerCommand("extendHopper", new Hopper().oscillateHopper());
    
    // new EventTrigger("runIntake").whileTrue(shooter.startIntakeCycle());

    
    
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
    driverXbox.leftBumper().whileTrue(new ShootCycle(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), () -> driverXbox.a().getAsBoolean(), shooter)).onFalse(shooter.stopCycles());
    driverXbox.axisGreaterThan(3, .1).whileTrue(hopper.hopperIn(() -> driverXbox.getRightTriggerAxis())).onFalse(hopper.stopHopper());
    driverXbox.axisGreaterThan(2, .1).whileTrue(hopper.hopperOut(()-> driverXbox.getLeftTriggerAxis())).onFalse(hopper.stopHopper());
    driverXbox.rightBumper().whileTrue(new HubAlign(swerveSubsystem, driverXbox));
    // driverXbox.b().whileTrue(shooter.testShooter()).onFalse(shooter.stopShooter());
    driverXbox.x().whileTrue(shooter.startFeedShooter()).onFalse(new InstantCommand(() -> shooter.stopFeed()));
    driverXbox.povUp().whileTrue(new SequentialCommandGroup(
      hopper.oscillationPrep(),
      hopper.oscillateHopper().repeatedly().withTimeout(5)
    ).repeatedly()).onFalse(hopper.stopHopper()); 
    driverXbox.a().whileTrue(shooter.ejectFuel()).onFalse(shooter.stopCycles());
    driverXbox.y().whileTrue(shooter.startIntakeCycle()).onFalse(shooter.stopIntakeCycle());
    driverXbox.povDown().whileTrue(new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), shooter)).onFalse(shooter.stopCycles());
    // Operator Controllers
    // opXbox.leftBumper().whileTrue(shooter.testShooterIntake()).onFalse(shooter.stopShooterIntake());
    // opXbox.rightBumper().whileTrue(shooter.startIntakeCycle()).onFalse(shooter.stopCycles());

     }


  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        AutoBuilder.buildAuto("side")
    );

  }
  
  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
