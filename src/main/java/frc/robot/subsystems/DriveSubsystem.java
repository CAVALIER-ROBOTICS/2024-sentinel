// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CycloidLibrary.NeoSteveModule;

public class DriveSubsystem extends SubsystemBase {
  NeoSteveModule fleft, fright, bleft, bright;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    fleft = new NeoSteveModule(Constants.fleft_drive_id, Constants.fright_steer_id, Constants.fleft_cancoder_id, 0);
    fright = new NeoSteveModule(Constants.fright_drive_id, Constants.fleft_steer_id, Constants.fright_cancoder_id, 0);
    bleft = new NeoSteveModule(Constants.bleft_drive_id, Constants.bleft_steer_id, Constants.bleft_cancoder_id, 0);
    bright = new NeoSteveModule(Constants.bright_drive_id, Constants.bright_steer_id, Constants.bright_cancoder_id, 0);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    fleft.setModuleState(states[0]);
    fright.setModuleState(states[1]);
    bleft.setModuleState(states[2]);
    bright.setModuleState(states[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = Constants.m_kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
