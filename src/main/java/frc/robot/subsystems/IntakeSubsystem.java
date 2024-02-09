// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */

    public CANSparkMax intakeLeft = new CANSparkMax(Constants.LEFT_INTAKE_ID , MotorType.kBrushless);
    CANSparkMax intakeRight = new CANSparkMax(Constants.RIGHT_INTAKE_ID, MotorType.kBrushless);
    CANSparkMax intakeSpin = new CANSparkMax(Constants.SPIN_INTAKE_ID, MotorType.kBrushless);

    PIDController pid = new PIDController(0, 0, 0);
    DutyCycleEncoder enc = new DutyCycleEncoder(0);

    ColorSensorV3 csv3 = new ColorSensorV3(Constants.COLOR_PORT);

    public boolean isExtended = false;

  public IntakeSubsystem() {

    //enc.setPositionOffset(0);
    intakeLeft.setInverted(true);
    intakeRight.follow(intakeLeft, true);
    intakeSpin.setIdleMode(IdleMode.kBrake);
    intakeRight.setIdleMode(IdleMode.kBrake);
    intakeLeft.setIdleMode(IdleMode.kBrake);

  }

  public double getAbsolutePosition() {
    return enc.getAbsolutePosition();
  }

  public boolean getIsUp() {
    return Math.abs(Constants.retractedPos - getAbsolutePosition()) < .1;
  }

  public void setPercentOutput(double point) {
    intakeLeft.set(point);
    intakeRight.set(-point);
  }

  public void setIntakeSpin(double point) {
    intakeSpin.set(point);
  }

  public double getProximity() {
    return csv3.getProximity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // setPos();
    SmartDashboard.putNumber("Intake Encoder" , enc.getAbsolutePosition());
  }
}
