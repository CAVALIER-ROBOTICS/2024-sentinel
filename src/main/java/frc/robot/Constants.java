// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int SPEED_SHOOTER = 1;
  public static final int MAX_POSITION_SHOOTER = 1; //FIXME
  public static final int MIN_POSITITON_SHOOTER = 0; //FIXME

  public static final int LEFT_INTAKE_ID = 16;
  public static final int RIGHT_INTAKE_ID = 17;
  public static final int SPIN_INTAKE_ID = 18;

  public static final int LEFT_CLIMB = 20;
  public static final int RIGHT_CLIMB = 21;

  public static final double RETRACTED_POS = 0.783558944588974;
  public static final double EXTENDED_POS = .077075026926876;
  public static final double INTAKE_LINEUP_POSITION = .5; //FIXME FIND THESE VALUES
  public static final double SHOOTER_LINEUP_POSITION = .5; //FIXME FIND THESE VALUES
  public static final double MOVE_SPEED = 0.1;

  public static final double FLEFT_OFFSET = -2.876213977285577;
  public static final double FRIGHT_OFFSET = -2.311709047343661;
  public static final double BLEFT_OFFSET = -2.16444689170664;
  public static final double BRIGHT_OFFSET = -0.193281579273591;
  
  public static final int FLEFT_DRIVE_ID = 1;
  public static final int FLEFT_STEER_ID = 2;

  public static final int FRIGHT_DRIVE_ID = 3;
  public static final int FRIGHT_STEER_ID = 4;

  public static final int BLEFT_DRIVE_ID = 5;
  public static final int BLEFT_STEER_ID = 6;

  public static final int BRIGHT_DRIVE_ID = 7;
  public static final int BRIGHT_STEER_ID = 8;

  public static final int PIGEON = 19;
  public static final int FLEFT_CANCODER_ID = 20;
  public static final int FRIGHT_CANCODER_ID = 21;
  public static final int BLEFT_CANCODER_ID = 22;
  public static final int BRIGHT_CANCODER_ID = 23;

  public static final int RIGHT_SHOOTER_PIVOT_ID = 11;
  public static final int LEFT_SHOOTER_PIVOT_ID = 12;
  public static final int KICKER_ID = 13;
  public static final int BOTTOM_SHOOTER_ID = 14;
  public static final int TOP_SHOOTER_ID = 15;

  public static final I2C.Port COLOR_PORT = I2C.Port.kOnboard;
  public static final int MINIMUM_PROXIMITY_TRIGGER = 1400;
  
  public static final String CANIVORE = "OTHERCANIVORE";

  private static double L = .5;
  private static double W = .5;

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(L/2, W/2),
    new Translation2d(L/2, -W/2),
    new Translation2d(-L/2, W/2),
    new Translation2d(-L/2, -W/2)
  );


}
