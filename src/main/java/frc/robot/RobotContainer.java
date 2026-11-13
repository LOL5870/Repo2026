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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands.Align;
import frc.robot.commands.AutoCommands.AutoShoot;
import frc.robot.commands.AutoCommands.HubAlign;
import frc.robot.commands.AutoCommands.HubAlignAuto;
import frc.robot.commands.ShooterCommands.ShootCycle;
import frc.robot.commands.ShooterCommands.ShooterCycleMan;
import frc.robot.commands.ShooterCommands.UnstuckBall;
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
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
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
    shooterTreeMap.put(0.905, 3350.0);
    shooterTreeMap.put(0.639, 3425.0);
    shooterTreeMap.put(0.462, 3525.0);
    shooterTreeMap.put(0.375, 3625.0);
    shooterTreeMap.put(0.315, 3650.0);
    shooterTreeMap.put(0.275, 3700.0);
    shooterTreeMap.put(0.247, 3725.0);
    shooterTreeMap.put(0.235, 3900.0);

    NamedCommands.registerCommand("shootCycleMiddle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(3.5));
    NamedCommands.registerCommand("shootCycle", new AutoShoot(() -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")), shooter).withTimeout(3.75));
    NamedCommands.registerCommand("hubAlignRight", new HubAlignAuto(swerveSubsystem));
    NamedCommands.registerCommand("hubAlignLeft", align);
    //NamedCommands.registerCommand("stopEverything", shooter.stopEverything(hopper));
    NamedCommands.registerCommand("runIntake", shooter.startIntakeCycle());
    NamedCommands.registerCommand("oscillateHopper", hopper.oscillateHopper());
    NamedCommands.registerCommand("extendHopper", hopper.hopperExtend());
    NamedCommands.registerCommand("testShooter", new InstantCommand(() -> System.out.println("shooting")).withTimeout(3));
    NamedCommands.registerCommand("testIntake", new InstantCommand(() -> System.out.println("intaking")));
    
    
    // Configure Bindings
    configureBindings();

    sendableChooser.setDefaultOption("nothing", null);
    sendableChooser.addOption("Left Auto", AutoBuilder.buildAuto("LeftAuto"));
    sendableChooser.addOption("Right Auto", AutoBuilder.buildAuto("RightAuto"));
    sendableChooser.addOption("middle Auto", AutoBuilder.buildAuto("MiddleAuto"));
    //sendableChooser.addOption("Left Over Auto", AutoBuilder.buildAuto("OverTrenchLeft"));
    sendableChooser.addOption("Left with depot Auto", AutoBuilder.buildAuto("LeftAutoWdepot"));
    sendableChooser.addOption("middle basic Auto", AutoBuilder.buildAuto("BasicMiddleAuto"));
    sendableChooser.addOption("Left Auto Extend", AutoBuilder.buildAuto("LeftAutoExtend"));
    sendableChooser.addOption("Right Auto Extend", AutoBuilder.buildAuto("RightAutoExtend"));

    SmartDashboard.putData("Auto chooser", sendableChooser);

    DriverStation.silenceJoystickConnectionWarning(true); // Get rid of controller error
  }

  private void configureBindings() {

    // Setup swervedrive command
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // Setup Controls
    // -- DRIVER CONTROLLER -- 
    // Zero Gyro when you press start button

    // WHILE pressing RIGHT BUMPER align to hub, Use BUTTON B as Boolean Supplier.

    // WHILE pressing LEFT BUMPER oscillate the hopper REPEATEDLY. 

    // WHILE pressing B SPIN INDEXER. 


    // -- OPERATOR XBOX --
    // WHILE pressing LEFT BUMPER run SHOOT CYCLE from SHOOTER COMMANDS FOLDER. 
    // EXAMPLE, DELETE THIS AFTER LOOKING THROUGH IT AND TRY WRITING IT. 
    opXbox.leftBumper().whileTrue(
      new ShootCycle(
        () -> shooterTreeMap.get(LimelightHelpers.getTA("limelight")),
        shooter, 
        () -> opXbox.rightBumper().getAsBoolean()
        ))
        .onFalse(shooter.stopCycles());
    
    // WHILE pressing D-PAD LEFT DETRACT HOPPER at 40% speed.

    // WHILE pressing D-PAD RIGHT EXTEND HOPPER at 40% speed. 

    // WHILE pressing Y start the INTAKE CYCLE. 

    // pressing a should eject the fuel. 

    // pressing x should get the ball unstuck. spin shooter in reverse at 4000 RPM

  }


  public Command getAutonomousCommand() {

    return sendableChooser.getSelected();
    // return hopper.hopperExtend(); 

  }
  
  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
