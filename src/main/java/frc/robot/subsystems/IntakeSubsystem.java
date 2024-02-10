// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */

    CANSparkMax intakeLeft = new CANSparkMax(Constants.LEFT_INTAKE_ID , MotorType.kBrushless);
    CANSparkMax intakeRight = new CANSparkMax(Constants.RIGHT_INTAKE_ID, MotorType.kBrushless);
    CANSparkMax intakeSpin = new CANSparkMax(Constants.SPIN_INTAKE_ID, MotorType.kBrushless);

    PIDController controller = new PIDController(1.5, 0.0001, 0.05);
    // PIDController upController= new PIDController(1.75, .0001, .05);
    // ArmFeedforward feedforward = new ArmFeedforward(1.95, .01);
    DutyCycleEncoder enc = new DutyCycleEncoder(0);

    ColorSensorV3 csv3 = new ColorSensorV3(Constants.COLOR_PORT);
    DigitalInput breamBreak = new DigitalInput(3);

    double currentGoalPos = getAbsolutePosition();

  public IntakeSubsystem() {
    // controller.setTolerance(.1);
    //enc.setPositionOffset(0);
    // intakeLeft.setInverted(true);
    intakeSpin.setIdleMode(IdleMode.kBrake);
    intakeRight.setIdleMode(IdleMode.kBrake);
    intakeLeft.setIdleMode(IdleMode.kBrake);

    // intakeRight.follow(intakeLeft, true);
    controller.enableContinuousInput(0, 1);

  }

  public double getAbsolutePosition() {
    return enc.getAbsolutePosition();
  }

  public boolean getIsUp() {
    return Math.abs(Constants.RETRACTED_POS - getAbsolutePosition()) < .1;
  }

  public void setAnglePercentOutput(double point) {
    intakeLeft.set(point);
    intakeRight.set(-point);
  }

  public void setIntakeSpin(double point) {
    intakeSpin.set(point);
  }

  public double getProximity() {
    return csv3.getProximity();
  }

  public void setPosition(double position) {
    double setpoint = controller.calculate(getAbsolutePosition(), position);
    setAnglePercentOutput(-setpoint);
  }

  public void setGoalPos(double pos) {
    currentGoalPos = pos;
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Encoder" , enc.getAbsolutePosition());
    SmartDashboard.putNumber("Sensor Proximty", csv3.getProximity());
    // controller.setP(SmartDashboard.getNumber("Intake angle P", 0));
    // SmartDashboard.putNumber("Current P", controller.getP());
    // goToAbsolutePos(currentGoalPos);
  }
}
