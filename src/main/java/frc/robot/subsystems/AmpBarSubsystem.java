// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpBarSubsystem extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(Constants.AmpBarConstants.AMP_MOTOR_ID, MotorType.kBrushless);
  PIDController controller = new PIDController(1.0, 0, 0);
  DutyCycleEncoder encoder = new DutyCycleEncoder(3);

  /** Creates a new AmpBarSubsystem. */
  public AmpBarSubsystem() {
    motor.setSmartCurrentLimit(35);
    motor.setIdleMode(IdleMode.kBrake);
  }

  public void set(double speed) {
    motor.set(speed);
  }

  public double getPosition() {
    return encoder.getAbsolutePosition();
  }

  public void setPosition(double position) {
    double speed = controller.calculate(getPosition(), position);
    set(speed);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AmpBarPos", getPosition());
    SmartDashboard.putBoolean("AtSetpoint", atSetpoint());
    // This method will be called once per scheduler run
  }
}
