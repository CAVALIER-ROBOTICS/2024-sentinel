// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int LEFT_INTAKE_ID = 16;
  public static final int RIGHT_INTAKE_ID = 17;
  public static final int SPIN_INTAKE_ID = 18;

  public static final double retractedPos = 0;
  public static final double extendedPos = .7;
  

  public static final int FLEFT_DRIVE_ID = 1;
  public static final int FLEFT_STEER_ID = 2;

  public static final int FRIGHT_DRIVE_ID = 3;
  public static final int FRIGHT_STEER_ID = 4;

  public static final int bleft_drive_id = 5;
  public static final int bleft_steer_id = 6;

  public static final int bright_drive_id = 7;
  public static final int bright_steer_id = 8;

  // public static final int fleft_cancoder_id = 10;
  // public static final int fright_cancoder_id = 11;
  // public static final int bleft_cancoder_id = 12;
  // public static final int bright_cancoder_id = 13;

  public static final int right_shooter_pivot = 11;
  public static final int left_shooter_pivot = 12;
  public static final int kicker = 13;
  public static final int bottom_shooter_id = 14;
  public static final int top_shooter_id = 15;
  

  private static double L = .5;
  private static double W = .5;

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(L/2, W/2),
    new Translation2d(-L/2, W/2),
    new Translation2d(L/2, -W/2),
    new Translation2d(-L/2, -W/2)
  );


}
