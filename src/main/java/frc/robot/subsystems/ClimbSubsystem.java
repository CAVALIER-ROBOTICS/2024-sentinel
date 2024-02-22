// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  
  CANSparkMax leftClimb = new CANSparkMax(Constants.LEFT_CLIMB_ID, MotorType.kBrushless);
  CANSparkMax rightClimb = new CANSparkMax(Constants.RIGHT_CLIMB_ID, MotorType.kBrushless);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    leftClimb.setInverted(false);
    rightClimb.setInverted(true);
    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setIdleMode(IdleMode.kBrake);
  }

  public void setLeft(double percent) {

    leftClimb.set(percent);
  } 

  public void setRight(double percent) {

     rightClimb.set(percent);
 }

  @Override
  public void periodic() {}
}

