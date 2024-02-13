// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  CANSparkMax leftClimb = new CANSparkMax(Constants.LEFT_CLIMB, MotorType.kBrushless);
  CANSparkMax rightClimb = new CANSparkMax(Constants.RIGHT_CLIMB, MotorType.kBrushless);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    leftClimb.setInverted(Constants.LEFT_INTAKE_DIRECTION);
    rightClimb.setInverted(Constants.RIGHT_INTAKE_DIRECTION);
  }

  public void setInverted()
  {
     leftClimb.setInverted(!leftClimb.getInverted());
     rightClimb.setInverted(!rightClimb.getInverted());
  }
  public void setNormal()
  {
    leftClimb.setInverted(Constants.LEFT_INTAKE_DIRECTION);
    rightClimb.setInverted(Constants.RIGHT_INTAKE_DIRECTION);
  }


  
  public void setLeftPercentOutput(double percent) {
    leftClimb.set(percent);
  } 

  public void setRightPercentOutput(double percent)
  {
    rightClimb.set(percent);
  }


  @Override
  public void periodic() {}
}

