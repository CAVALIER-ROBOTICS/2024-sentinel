// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax top = new CANSparkMax(Constants.top_shooter_id, MotorType.kBrushless);
  CANSparkMax bottom = new CANSparkMax(Constants.bottom_shooter_id, MotorType.kBrushless);
  CANSparkMax left = new CANSparkMax(Constants.left_shooter_pivot, MotorType.kBrushless);
  CANSparkMax right = new CANSparkMax(Constants.right_shooter_pivot, MotorType.kBrushless);

  DutyCycleEncoder enc = new DutyCycleEncoder(0);

  double maxPos = 0;
  double minPos = 0;

  PIDController angler = new PIDController(0, 0, 0);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    left.setInverted(true);
    right.setInverted(false);
  }

  /*Formula for going from degrees to setpoint

   ((theta/90) * (max-min)) + min
   90 is the range of motion
   
   */

  public void setSpeed(double speed) {
    top.set(speed);
    bottom.set(speed);
  }

  public void setAngle(double theta) {

    left.set(angler.calculate(enc.getAbsolutePosition(), angleToSetPoints(theta)));
    right.set(angler.calculate(enc.getAbsolutePosition(), angleToSetPoints(theta)));
  }

  public double angleToSetPoints(double theta) {

// 90 is the range of motion in degrees
    return ((theta/90) * (maxPos - minPos)) + minPos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
