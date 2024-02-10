// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax top = new CANSparkMax(Constants.TOP_SHOOTER_ID, MotorType.kBrushless);
  CANSparkMax bottom = new CANSparkMax(Constants.BOTTOM_SHOOTER_ID, MotorType.kBrushless);
  CANSparkMax left = new CANSparkMax(Constants.LEFT_SHOOTER_PIVOT_ID, MotorType.kBrushless);
  CANSparkMax right = new CANSparkMax(Constants.RIGHT_SHOOTER_PIVOT_ID, MotorType.kBrushless);
  CANSparkMax kicker = new CANSparkMax(Constants.KICKER_ID, MotorType.kBrushless);

  DigitalInput limit = new DigitalInput(Constants.SHOOTER_LIMIT_SWITCH_ID);

  DutyCycleEncoder enc = new DutyCycleEncoder(1);
  PIDController angleController = new PIDController(1.5, 0.0001, 0.05);

  RelativeEncoder rpmEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // angleController.setPositionTolerance(.001);
    angleController.enableContinuousInput(0, 1);

    top.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    bottom.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);
    top.setIdleMode(IdleMode.kCoast);
    bottom.setIdleMode(IdleMode.kCoast);
    // right.follow(left, true);
    bottom.follow(top, true);

    rpmEncoder = top.getEncoder();
  }

  public void setFlywheelSpeed(double speed) {
    top.set(speed);
    // bottom.set(speed);
  }

  public void setAngleSpeed(double speed) {
    SmartDashboard.putNumber("Requested shooter speed", speed);
    left.set(speed);
    right.set(-speed);
  }

  public void setKickerSpeed(double speed) {
    kicker.set(speed);
  }

  public double getAbsolutePosition() {
    return enc.getAbsolutePosition();
  }

  public void setPosition(double position) {
    double setpoint = angleController.calculate(getAbsolutePosition(), position);
    setAngleSpeed(-setpoint);
  }

  public boolean atSetpoint() {
    return angleController.atSetpoint();
  }

  public boolean hasNoteInShooter() {
    return limit.get();
  }

  public double getRPM() {
    return rpmEncoder.getVelocity();
  }
  /*Formula for going from degrees to setpoint

  ((theta/90) * (max-min)) + min
  90 is the range of motion
   
  */


//   public void setAngle(double theta) {

//     left.set(angler.calculate(enc.getAbsolutePosition(), angleToSetPoints(theta)));
//     right.set(angler.calculate(enc.getAbsolutePosition(), angleToSetPoints(theta)));
//   }

//   public double angleToSetPoints(double theta) {

// // 90 is the range of motion in degrees
//     return ((theta/90) * (maxPos - minPos)) + minPos;
//   }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AbsoluteShooterPosition", getAbsolutePosition());
    // This method will be called once per scheduler run
  }
}
