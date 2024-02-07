// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax top = new CANSparkMax(Constants.top_shooter_id, MotorType.kBrushless);
  CANSparkMax bottom = new CANSparkMax(Constants.bottom_shooter_id, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  public void setSpeed(double speed) {
    top.set(speed);
    bottom.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
