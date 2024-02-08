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

    leftClimb.setInverted(true);
    rightClimb.setInverted(false);
  }

  public void raiseClimb() {

    leftClimb.set(0);
    rightClimb.set(0);
  }

  public void lowerClimb() {

    leftClimb.set(0);
    rightClimb.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
