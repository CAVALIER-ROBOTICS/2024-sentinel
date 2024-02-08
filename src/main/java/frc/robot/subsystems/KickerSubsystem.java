// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new Kicker. */
  CANSparkMax kick = new CANSparkMax(Constants.kicker, MotorType.kBrushless);

  public KickerSubsystem() {

  }

  public void runKicker() {

    kick.set(0);
  }

  public void stopKick() {

    kick.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
