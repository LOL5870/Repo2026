package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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

        shooterConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(1000)
        .maxAcceleration(1000)
        .allowedProfileError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedProfileError(1, ClosedLoopSlot.kSlot1);
        // Apply the configurations
        shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterIntake.configure(shooterIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indxr.configure(indxrConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        groundIntake.configure(groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialzing the widgets
        SmartDashboard.setDefaultNumber("Shooter Speed", 0);
        SmartDashboard.setDefaultNumber("ShooterIntake Speed", 0);
        SmartDashboard.setDefaultNumber("Indxr speed", -0.4);
        SmartDashboard.setDefaultNumber("GroundIntake Speed", 0); 
    }

    // printing out the speeds of the motors to elastic
    public Command testShooter(){
        return run(()-> shooter.set(SmartDashboard.getNumber("Shooter Speed", 0)));
    }

    public Command testShooterIntake(){
        return run(()-> shooterIntake.set(SmartDashboard.getNumber("ShooterIntake Speed", 0)));
    }

    public Command testIndxr(){
        return run(()-> indxr.set(SmartDashboard.getNumber("Indxr speed", -0.4)));
    }

    public Command testGroundIntake(){
        return run(()-> groundIntake.set(SmartDashboard.getNumber("GroundIntake Speed", 0)));
    }

    @Override
    public void periodic() {
        // groundIntake.set(SmartDashboard.getNumber("GroundIntake Speed", 0));
        // indxr.set(SmartDashboard.getNumber("Indxr Speed", 0));
        // shooterIntake.set(SmartDashboard.getNumber("ShooterIntake Speed", 0));
        // shooter.set(SmartDashboard.getNumber("Shooter Speed", 0));
        SmartDashboard.putNumber("Actual Position", shooterEncoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", shooterEncoder.getVelocity());
    }

    public Command stopIndxr() { 
        return runOnce(() -> indxr.set(0));
    }

    public Command stopShooterIntake() { 
        return runOnce(() -> shooterIntake.set(0));
    }

    public Command stopGroundIntake() { 
        return runOnce(() -> groundIntake.set(0));
    }

    public Command stopShooter() { 
        return runOnce(() -> shooter.set(0));
    }

    public Command startIntakeCycle(){
        return run(()->{
            shooterIntake.set(-0.5);
            indxr.set(-0.4);
            groundIntake.set(-0.5);
        });

    }

    public Command shootCycle(){
        return run(()->{
            shooterIntake.set(-0.9);
            indxr.set(0.6);

        });
    }

    public Command ejectFuel(){
        return run(() ->{
            shooterIntake.set(0.5);
            indxr.set(0.4);

        });
    }

    public Command stopCycles(){
        return run(()->{
            shooterIntake.set(0);
            indxr.set(0);
            groundIntake.set(0);
        });
    }
}