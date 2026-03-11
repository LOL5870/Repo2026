package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterIntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {
    
    // Define Motors
    public SparkMax shooter; 
    public SparkMax shooterIntake;
    public SparkMax indxr; 
    SparkMax groundIntake; 

    // Define Configurations
    public SparkMaxConfig shooterConfig; 
    public SparkMaxConfig shooterIntakeConfig; 
    public SparkMaxConfig indxrConfig; 
    SparkMaxConfig groundIntakeConfig; 
    
    // Define Encoders
    RelativeEncoder shooterIntakeEncoder; 
    RelativeEncoder shooterEncoder;    

    // Define closed loop controllers
    public SparkClosedLoopController shooterIntakeController;
    public SparkClosedLoopController shooterController; 


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

        // Configure the Motors        
        indxrConfig.closedLoop.outputRange(ShooterIntakeConstants.INDXR_MIN_SPEED, ShooterIntakeConstants.INDXR_MAX_SPEED);
        groundIntakeConfig.closedLoop.outputRange(ShooterIntakeConstants.GROUND_MIN_SPEED, ShooterIntakeConstants.GROUND_MAX_SPEED);
        
        shooterConfig.closedLoop
        .outputRange(ShooterIntakeConstants.SHOOTER_MIN_SPEED, ShooterIntakeConstants.SHOOTER_MAX_SPEED)
        .pid(ShooterIntakeConstants.shooterPID.kP, ShooterIntakeConstants.shooterPID.kI, ShooterIntakeConstants.shooterPID.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)      .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(.0021);
        
        shooterIntakeConfig.closedLoop
        .outputRange(ShooterIntakeConstants.SHOOTER_MIN_SPEED, ShooterIntakeConstants.SHOOTER_MAX_SPEED)
        .pid(ShooterIntakeConstants.shooterIntakePID.kP, ShooterIntakeConstants.shooterIntakePID.kI, ShooterIntakeConstants.shooterIntakePID.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(.0022);

        shooterConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(1000)
        .maxAcceleration(4000)
        .allowedProfileError(0.5)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        shooterIntakeConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(1000)
        .maxAcceleration(4000)
        .allowedProfileError(.5)
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
    
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Intake Velocity", shooterIntakeEncoder.getVelocity());

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

    // printing out the speeds of the motors to elastic
    public Command testShooter(){
        return run(()-> {
            shooterController.setSetpoint(SmartDashboard.getNumber("Shooter Speed", 0), ControlType.kMAXMotionVelocityControl);
        
        }); 
    }

    public Command testShooterIntake(){
        return run(()-> shooterIntakeController.setSetpoint(-SmartDashboard.getNumber("ShooterIntake Speed", 0), ControlType.kMAXMotionVelocityControl));

    }

    public Command startIntakeCycle(){
        return run(()->{
            shooterIntake.set(-0.5);
            indxr.set(-0.4);
            groundIntake.set(-0.5);
        });

    }

    public Command shootCycle(Supplier<Double> shooterIntakeDist, Supplier<Double> shooterDist){

        return run(()->{
            shooterIntakeController.setSetpoint(-shooterIntakeDist.get(), ControlType.kMAXMotionVelocityControl); 
            shooterController.setSetpoint(shooterDist.get(), ControlType.kMAXMotionVelocityControl);
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
            shooterIntake.stopMotor();
            shooter.stopMotor();
            indxr.stopMotor();
            groundIntake.stopMotor();
        });
    }

    public void startIndxr() {
        indxr.set(0.6);
    }

    public void setShooterRPM(Supplier<Double> rpm){
        shooterController.setSetpoint(rpm.get(), ControlType.kMAXMotionVelocityControl);
    }
    
    public double getShooterRPM(){
        return shooterEncoder.getVelocity();
    }


    public void setIntakeRPM(Supplier<Double> rpm){
        shooterIntakeController.setSetpoint(rpm.get(), ControlType.kMAXMotionVelocityControl);
    }

    public double getShooterIntakeRPM(){
        return shooterIntakeEncoder.getVelocity();
    }

    public void feedFuel(){
        indxr.set(0.6);
    }

    public void stopFeed(){
        indxr.stopMotor();
    }
}