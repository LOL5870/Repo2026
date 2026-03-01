package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterIntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {
    
    // Define Motors
    SparkMax shooter; 
    SparkMax shooterIntake;
    SparkMax indxr; 
    SparkMax groundIntake; 

    // Define Configurations
    SparkMaxConfig shooterConfig; 
    SparkMaxConfig shooterIntakeConfig; 
    SparkMaxConfig indxrConfig; 
    SparkMaxConfig groundIntakeConfig; 
    
    // Define Encoders
    RelativeEncoder shooterIntakeEncoder; 
    RelativeEncoder shooterEncoder;    

    // Define closed loop controllers
    SparkClosedLoopController shooterIntakeController;
    SparkClosedLoopController shooterController; 

    // Tree maps
    InterpolatingDoubleTreeMap shooterIntakeTreeMap; 
    InterpolatingDoubleTreeMap shooterTreeMap; 
     
    

    public Shooter(){
        
        // Initialize the Motors
        shooter = new SparkMax(ShooterIntakeConstants.ShooterID, MotorType.kBrushless); 
        shooterIntake = new SparkMax(ShooterIntakeConstants.ShooterIntakeID, MotorType.kBrushless); 
        indxr = new SparkMax(ShooterIntakeConstants.IndxrID, MotorType.kBrushless); 
        groundIntake = new SparkMax(ShooterIntakeConstants.GroundIntakeID, MotorType.kBrushless);
        
        // Inialize Configuratons
        shooterConfig = new SparkMaxConfig();
        shooterIntakeConfig = new SparkMaxConfig();
        indxrConfig = new SparkMaxConfig();
        groundIntakeConfig = new SparkMaxConfig();

        // Initialize the encoders
        shooterEncoder = shooter.getEncoder(); 
        shooterIntakeEncoder = shooterIntake.getEncoder(); 

        // Initialize the Closed loop controllers
        shooterController = shooter.getClosedLoopController();
        shooterIntakeController = shooterIntake.getClosedLoopController();

        // Initialize the tree maps
        shooterIntakeTreeMap = new InterpolatingDoubleTreeMap(); 
        shooterTreeMap = new InterpolatingDoubleTreeMap();

        // Configure the Motors        
        indxrConfig.closedLoop.outputRange(ShooterIntakeConstants.INDXR_MIN_SPEED, ShooterIntakeConstants.INDXR_MAX_SPEED);
        groundIntakeConfig.closedLoop.outputRange(ShooterIntakeConstants.GROUND_MIN_SPEED, ShooterIntakeConstants.GROUND_MAX_SPEED);
        
        shooterConfig.closedLoop
        .outputRange(ShooterIntakeConstants.SHOOTER_MIN_SPEED, ShooterIntakeConstants.SHOOTER_MAX_SPEED)
        //.pid(ShooterIntakeConstants.shooterPID.kP, ShooterIntakeConstants.shooterPID.kI, ShooterIntakeConstants.shooterPID.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        shooterIntakeConfig.closedLoop
        .outputRange(ShooterIntakeConstants.SHOOTER_MIN_SPEED, ShooterIntakeConstants.SHOOTER_MAX_SPEED)
        //.pid(ShooterIntakeConstants.shooterIntakePID.kP, ShooterIntakeConstants.shooterIntakePID.kI, ShooterIntakeConstants.shooterIntakePID.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // Apply the configurations
        shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterIntake.configure(shooterIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indxr.configure(indxrConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        groundIntake.configure(groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialzing the widgets
        SmartDashboard.setDefaultNumber("Shooter Speed", 0);
        SmartDashboard.setDefaultNumber("ShooterIntake Speed", 0);
        SmartDashboard.setDefaultNumber("Indxr speed", 0);
        SmartDashboard.setDefaultNumber("GroundIntake Speed", 0); 
    }

    // Examplar code for testing
    public Command testShooter(){
        return run(()-> shooter.set(SmartDashboard.getNumber("Shooter Speed", 0)));
    }

    @Override
    public void periodic() {

    }

    

}