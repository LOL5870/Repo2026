// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.AutoCommands.Align;
import frc.robot.commands.swervedrive.AutoCommands.AutoShoot;
import frc.robot.commands.swervedrive.AutoCommands.HubAlign;
import frc.robot.commands.swervedrive.ShooterCommands.ShootCycle;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import swervelib.SwerveInputStream;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

  // Initialize Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opXbox = new CommandXboxController(1);
  InterpolatingDoubleTreeMap  shooterIntakeTreeMap = new InterpolatingDoubleTreeMap(); 
  InterpolatingDoubleTreeMap  shooterTreeMap = new InterpolatingDoubleTreeMap();
  Hopper hopper = new Hopper();


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

        // ShooterIntake tree map
        shooterIntakeTreeMap.put(0.965, 3550.0); 
        shooterIntakeTreeMap.put(0.765, 3600.0);
        shooterIntakeTreeMap.put(0.545, 3650.0); 
        shooterIntakeTreeMap.put(0.375, 3900.0); 
        shooterIntakeTreeMap.put(0.315, 3975.0); 
        shooterIntakeTreeMap.put(0.235, 4125.0); 
  
        // Shooter tree map
        shooterTreeMap.put(0.965, 3500.0); 
        shooterTreeMap.put(0.765, 3550.0); 
        shooterTreeMap.put(0.545, 3675.0);
        shooterTreeMap.put(0.375, 3825.0); 
        shooterTreeMap.put(0.315, 3900.0); 
        shooterTreeMap.put(0.235, 4050.0); 


        NamedCommands.registerCommand("shootCycleMiddle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), () -> shooterIntakeTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(7));
        NamedCommands.registerCommand("shootCycle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), () -> shooterIntakeTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(8));
        NamedCommands.registerCommand("hubAlign", new HubAlign(swerveSubsystem, driverXbox));


    // Configure Bindings
    configureBindings();

    // Setup optional paths
    // sendableChooser.setDefaultOption("Nothing", null);
    // sendableChooser.addOption("To source and back", new AutoShoot(swerveSubsystem, shooter));

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
    driverXbox.leftBumper().whileTrue(new ShootCycle(() -> shooterIntakeTreeMap.get(LimelightHelpers.getTA("limelight")), () -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")),() ->  driverXbox.a().getAsBoolean(), shooter)).onFalse(shooter.stopCycles());
    driverXbox.axisGreaterThan(3, .1).whileTrue(hopper.hopperIn(() -> driverXbox.getRightTriggerAxis())).onFalse(hopper.stopHopper());
    driverXbox.axisGreaterThan(2, .1).whileTrue(hopper.hopperOut(()-> driverXbox.getLeftTriggerAxis())).onFalse(hopper.stopHopper());
    driverXbox.rightBumper().whileTrue(new HubAlign(swerveSubsystem, driverXbox));

    // Operator Controllers
    // opXbox.a().whileTrue(shooter.testGroundIntake()).onFalse(shooter.stopGroundIntake());
    // opXbox.rightBumper().whileTrue(shooter.startIntakeCycle()).onFalse(shooter.stopCycles());

  }


  public Command getAutonomousCommand() {
      return new SequentialCommandGroup(

        AutoBuilder.buildAuto(null)
      );
  }
  
  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
