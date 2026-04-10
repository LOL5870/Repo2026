// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static final class DrivebaseConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class ShooterIntakeConstants { 
    // IDS
    public static final int ShooterRightID = 14; 
    public static final int ShooterLeftID = 10; 
    public static final int GroundIntakeID = 13;
    public static final int IndxrID = 12; 
    public static final int IntakeFlapsID = 11; 

    // MIN SPEEDS
    public static final double SHOOTER_MIN_SPEED = -1; 
    public static final double INDXR_MIN_SPEED = -1; 
    public static final double GROUND_MIN_SPEED = -1; 
    public static final double INTAKEFLAPS_MIN_SPEED = -1; 

    // MAX SPEEDS
    public static final double SHOOTER_MAX_SPEED = 0.9; 
    public static final double INDXR_MAX_SPEED = 0.9; 
    public static final double GROUND_MAX_SPEED = 0.9; 
    public static final double INTAKEFLAPS_MAX_SPEED = 0.9; 


    // PID
    public static final PIDConstants shooterPID = new PIDConstants(0.00010, 0.0, 0.0);
    public static final PIDConstants shooter2PID = new PIDConstants(0.000085,0,0);
    
  }

  public static class HopperConstants{

    // Change theses if needed
    public static final int rightMotorID = 6;
    public static final int leftMotorID = 7;

  }

  public static class AutoConstants{
    // PID
    public static final PIDConstants HUB_ALIGN_PID = new PIDConstants(0.08, 0,0);
    public static final PIDConstants HUB_ALIGN_PID_TELE = new PIDConstants(0.08, 0,0);
    
    // x Offset for hub
    public static final double xOffsetSide = 1;
    public static final double xOffsetCorner = 1; 
  }

  public static enum TAGS {
      right(0),
      middle(1), 
      left(2);
  
    public final int value;

    TAGS(int value) {
      this.value = value;
    }
  }
  
  public static class AprilTagIDs {
    
    // Apple tags
    public static final int[] RED_HUB_APRIL_TAGS = {3, 10, 5};
    public static final int[] BLUE_HUB_APRIL_TAGS = {27, 26, 24};

  }
}
