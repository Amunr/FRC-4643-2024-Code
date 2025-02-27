// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kMaintenaceControllerPort =2;
  }
  public static class motorConstants {
    public static final int kIntakeMotorPort = 10;
    public static final int kIndexerLeftMotorPort = 11;
    public static final int kIndexerRightMotorPort = 12;
    public static final int kShooter = 13; 
    public static final int kclimberLeftPort = 14;
    public static final int kclimberRightPort = 15;
  }
  public static class portConstants {   
     public static final int shooterBeamBreak = 0;
  }
  public static class shooterConstants {
    public static double kProportoinal = 0.001;
    public static double kIntegral = 0;
    public static double kDerivative = 0;
    public static final  double kStaticGain = 0.05;
    public static final double kVoltage = 0.00115;
    public static final double kPIDTolerance = 0.05;
  }

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }
}