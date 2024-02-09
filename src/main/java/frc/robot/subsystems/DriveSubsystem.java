// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CycloidLibrary.NeoSteveModule;

public class DriveSubsystem extends SubsystemBase {
  NeoSteveModule fleft, fright, bleft, bright;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    fleft = new NeoSteveModule(Constants.FLEFT_DRIVE_ID, Constants.FLEFT_STEER_ID, Constants.fleft_cancoder_id, Constants.FLEFT_OFFSET, Constants.CANIVORE);
    fright = new NeoSteveModule(Constants.FRIGHT_DRIVE_ID, Constants.FRIGHT_STEER_ID, Constants.fright_cancoder_id, Constants.FRIGHT_OFFSET, Constants.CANIVORE);
    bleft = new NeoSteveModule(Constants.bleft_drive_id, Constants.bleft_steer_id, Constants.bleft_cancoder_id, Constants.BLEFT_OFFSET, Constants.CANIVORE);
    bright = new NeoSteveModule(Constants.bright_drive_id, Constants.bright_steer_id, Constants.bright_cancoder_id, Constants.BRIGHT_OFFSET, Constants.CANIVORE);
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
    SmartDashboard.putNumber("FLEFT", fleft.getEncoderPosition());
    SmartDashboard.putNumber("FRIGHT", fright.getEncoderPosition());
    SmartDashboard.putNumber("BLEFT", bleft.getEncoderPosition());
    SmartDashboard.putNumber("BRIGHT", bright.getEncoderPosition());
    // This method will be called once per scheduler run
  }
}
