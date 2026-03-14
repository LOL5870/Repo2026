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
import com.pathplanner.lib.auto.AutoBuilder; 
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

  // Initialize Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opXbox = new CommandXboxController(1);
  InterpolatingDoubleTreeMap  shooterTreeMap = new InterpolatingDoubleTreeMap();
   
  // Swerve Initialization
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  private SendableChooser<Command> sendableChooser = new SendableChooser<>();

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
    shooterTreeMap.put(0.545, 3625.0);
    shooterTreeMap.put(0.375, 3800.0); 
    shooterTreeMap.put(0.315, 3800.0); 
    shooterTreeMap.put(0.235, 3900.0); 

    NamedCommands.registerCommand("shootCycleMiddle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(3));
    NamedCommands.registerCommand("shootCycle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(7));
    NamedCommands.registerCommand("hubAlign", new HubAlign(swerveSubsystem, driverXbox));
    //NamedCommands.registerCommand("stopEverything", shooter.stopEverything(hopper));
    NamedCommands.registerCommand("runIntake", shooter.startIntakeCycle());
    NamedCommands.registerCommand("oscillateHopper", hopper.oscillateHopper());
    NamedCommands.registerCommand("extendHopper", hopper.hopperExtend());
    
    
    // Configure Bindings
    configureBindings();

    sendableChooser.setDefaultOption("nothing", null);
    sendableChooser.addOption("Left Auto", AutoBuilder.buildAuto("LeftAuto"));
    sendableChooser.addOption("Right Auto", AutoBuilder.buildAuto("RightAuto"));
    sendableChooser.addOption("middle Auto", AutoBuilder.buildAuto("MiddleAuto"));
    //sendableChooser.addOption("Left Over Auto", AutoBuilder.buildAuto("OverTrenchLeft"));

    SmartDashboard.putData("Auto chooser", sendableChooser);

    DriverStation.silenceJoystickConnectionWarning(true); // Get rid of controller error
  }

  private void configureBindings() {

    // Setup swervedrive command
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // Setup Controls
    // Driver Controller
    driverXbox.start().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    driverXbox.rightBumper().whileTrue(new HubAlign(swerveSubsystem, driverXbox));
    driverXbox.leftBumper().whileTrue(new SequentialCommandGroup( // jorker 5000
      hopper.oscillationPrep(),
      hopper.oscillateHopper().repeatedly().withTimeout(5)
    )).onFalse(hopper.stopHopper()); 


    opXbox.leftBumper().whileTrue(new ShootCycle(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), () -> opXbox.rightBumper().getAsBoolean(), shooter)).onFalse(shooter.stopCycles());
    opXbox.povLeft().whileTrue(hopper.hopperIn(() -> 0.4)).onFalse(hopper.stopHopper());
    opXbox.povRight().whileTrue(hopper.hopperOut(()-> 0.4)).onFalse(hopper.stopHopper());
    opXbox.axisGreaterThan(1, 0.1).whileTrue(hopper.spinLeftMotor(() -> -opXbox.getRawAxis(1))).onFalse(hopper.stopLeftMotor()); 
    opXbox.axisGreaterThan(5, 0.1).whileTrue(hopper.spinRightMotor(() -> -opXbox.getRawAxis(5))).onFalse(hopper.stopRightMotor());
    opXbox.y().whileTrue(shooter.startIntakeCycle()).onFalse(shooter.stopIntakeCycle()); 
    opXbox.a().whileTrue(shooter.ejectFuel()).onFalse(shooter.stopCycles());
    opXbox.povUp().whileTrue(shooter.fixedRPM(3800)).onFalse(shooter.stopShooter());
    opXbox.b().whileTrue(shooter.startFeedFuel()).onFalse(shooter.stopFeedFuel());
  }


  public Command getAutonomousCommand() {

    return sendableChooser.getSelected();

  }
  
  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
