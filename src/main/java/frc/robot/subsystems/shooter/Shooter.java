package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterIntakeConstants;
import frc.robot.subsystems.hopper.Hopper;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {
    
    // Define Motors
    public SparkMax shooterLeft; 
    public SparkMax shooterRight;
    public SparkMax indxr; 
    public SparkMax groundIntake; 
    public SparkMax intakeFlap;

    // Define Configurations
    public SparkMaxConfig shooterLeftConfig;
    public SparkMaxConfig shooterRightConfig; 
    public SparkMaxConfig indxrConfig; 
    public SparkMaxConfig groundIntakeConfig;
    public SparkMaxConfig intakeFlapConfig;
    
    // Define Encoders
    public RelativeEncoder shooterRightEncoder;
    public RelativeEncoder shooterLeftEncoder;

    // Define closed loop controllers
    public SparkClosedLoopController shooterRightController;
    public SparkClosedLoopController shooterLeftController;      
    

    public Shooter(){
        
        // Initialize the Motors
        shooterLeft = new SparkMax(ShooterIntakeConstants.ShooterLeftID, MotorType.kBrushless); 
        shooterRight = new SparkMax(ShooterIntakeConstants.ShooterRightID, MotorType.kBrushless); 
        indxr = new SparkMax(ShooterIntakeConstants.IndxrID, MotorType.kBrushless); 
        groundIntake = new SparkMax(ShooterIntakeConstants.GroundIntakeID, MotorType.kBrushless);
        intakeFlap = new SparkMax(ShooterIntakeConstants.IntakeFlapsID, MotorType.kBrushless);
        
        // Inialize Configuratons
        shooterLeftConfig = new SparkMaxConfig();
        shooterRightConfig = new SparkMaxConfig();
        indxrConfig = new SparkMaxConfig();
        groundIntakeConfig = new SparkMaxConfig();
        intakeFlapConfig = new SparkMaxConfig();

        // Initialize the encoders
        shooterLeftEncoder = shooterLeft.getEncoder(); 
        shooterRightEncoder = shooterRight.getEncoder(); 

        // Initialize the Closed loop controllers
        shooterLeftController = shooterLeft.getClosedLoopController();
        shooterRightController = shooterRight.getClosedLoopController();

        // Configure the Motors        
        indxrConfig.closedLoop.outputRange(ShooterIntakeConstants.INDXR_MIN_SPEED, ShooterIntakeConstants.INDXR_MAX_SPEED);
        groundIntakeConfig.closedLoop.outputRange(ShooterIntakeConstants.GROUND_MIN_SPEED, ShooterIntakeConstants.GROUND_MAX_SPEED);
        intakeFlapConfig.closedLoop.outputRange(ShooterIntakeConstants.INTAKEFLAPS_MIN_SPEED, ShooterIntakeConstants.INTAKEFLAPS_MAX_SPEED);

        shooterLeftConfig.closedLoop
        .outputRange(ShooterIntakeConstants.SHOOTER_MIN_SPEED, ShooterIntakeConstants.SHOOTER_MAX_SPEED)
        .pid(ShooterIntakeConstants.shooterPID.kP, ShooterIntakeConstants.shooterPID.kI, ShooterIntakeConstants.shooterPID.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)      
        .feedForward
        .kV(.0021);
        
        shooterLeftConfig.closedLoop.maxMotion
        .cruiseVelocity(1000)
        .maxAcceleration(4000)
        .allowedProfileError(0.5);

        shooterRightConfig.closedLoop
        .outputRange(ShooterIntakeConstants.SHOOTER_MIN_SPEED, ShooterIntakeConstants.SHOOTER_MAX_SPEED)
        .pid(ShooterIntakeConstants.shooter2PID.kP, ShooterIntakeConstants.shooter2PID.kI, ShooterIntakeConstants.shooter2PID.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)      
        .feedForward
        .kV(.00212);
        
        shooterRightConfig.closedLoop.maxMotion
        .cruiseVelocity(1000)
        .maxAcceleration(4000)
        .allowedProfileError(0.5);
        
        // Apply the configurations
        shooterLeft.configure(shooterLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterRight.configure(shooterRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indxr.configure(indxrConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        groundIntake.configure(groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeFlap.configure(intakeFlapConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialzing the widgets
        SmartDashboard.setDefaultNumber("Shooters Speed", 0);
        SmartDashboard.setDefaultNumber("Intake Flaps Speed", 0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RIGHT RPM", -shooterRightEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter LEFT RPM", shooterLeftEncoder.getVelocity());
    }   


    public Command stopIndxr() { 
        return runOnce(() -> indxr.stopMotor());
    }


    public Command stopGroundIntake() { 
        return runOnce(() -> groundIntake.stopMotor());
    }

    public Command stopShooter() { 
        return runOnce(() ->{
            
            shooterLeft.stopMotor(); 
            shooterRight.stopMotor();
        });
    }

    public Command stopIntakeFlaps() { 
        return runOnce(() -> intakeFlap.stopMotor());
    }

    public Command testShooter(){
        return run(()-> {
            shooterLeftController.setSetpoint(SmartDashboard.getNumber("Shooters Speed", 0), ControlType.kMAXMotionVelocityControl);
            shooterRightController.setSetpoint(-SmartDashboard.getNumber("Shooters Speed", 0), ControlType.kMAXMotionVelocityControl);
        }); 
    }

    public Command testIntakeFlaps(){
        return run(()-> intakeFlap.set(SmartDashboard.getNumber("Intake Flaps Speed", 0)));
    }

    public Command startIntakeCycle(){
        return run(()->{
            intakeFlap.set(-0.5);
            indxr.set(-0.4);
            groundIntake.set(-0.5);
        });
    }

    public Command stopIntakeCycle(){
        return run(()->{
            intakeFlap.stopMotor();
            indxr.stopMotor();
            groundIntake.stopMotor();
        });
    }

    public Command shootCycle(Supplier<Double> shooterLeftDist){
        return run(()->{
        shooterLeftController.setSetpoint(shooterLeftDist.get(), ControlType.kMAXMotionVelocityControl); 
        shooterRightController.setSetpoint(-shooterLeftDist.get(), ControlType.kMAXMotionVelocityControl); 
        });
    }

    public Command startFeedShooter() {
        return run(() ->{
            indxr.set(0.6);
            intakeFlap.set(-0.5);
        });
    }

    public Command startFeedFuel(){
        return run(() -> {
            indxr.set(0.6);
            intakeFlap.set(-0.5);

        });
    }

    public Command stopFeedFuel(){
        return run(() -> {
            indxr.stopMotor();
            intakeFlap.stopMotor();

        });
    }


    public Command ejectFuel(){
        return run(() ->{ 
            intakeFlap.set(0.5);
            indxr.set(0.5);

        });
    }

    public Command stopCycles(){
        return run(()->{
            shooterLeft.stopMotor();
            shooterRight.stopMotor();
            intakeFlap.stopMotor();
            indxr.stopMotor();
            groundIntake.stopMotor();
        });
    }

    public Command fixedRPM(double rpm){
        return run(()->{
            shooterLeftController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl); 
            shooterRightController.setSetpoint(-rpm, ControlType.kMAXMotionVelocityControl); 

        });

    }
    
    public void setRPM(double rpm){
        shooterLeftController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl); 
        shooterRightController.setSetpoint(-rpm, ControlType.kMAXMotionVelocityControl); 
    }

    public double getShooterRPM() {
        return shooterRightEncoder.getVelocity(); 
    }

    public void feedFuel() { 
        indxr.set(0.6);
        intakeFlap.set(-0.5);
    }
    
    public void stopFeed() { 
        indxr.stopMotor();
        intakeFlap.stopMotor();
    }

    public Command stopEverything(Hopper hopper) { 
        return runOnce(() -> {
            hopper.stopHopper(); 
            indxr.stopMotor();
            intakeFlap.stopMotor();
            groundIntake.stopMotor();
            shooterLeft.stopMotor();
            shooterRight.stopMotor();
        } 
        ); 
    }
    
}