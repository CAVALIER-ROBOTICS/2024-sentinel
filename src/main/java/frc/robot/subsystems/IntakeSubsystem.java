// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//If you have issues with the rev2mdistancesensor install, please reinstall it locally
// https://github.com/REVrobotics/2m-Distance-Sensor 
package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BasicLibrary.SmartMax;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */

    SmartMax intakeLeft = new SmartMax(Constants.LEFT_INTAKE_ID, IdleMode.kBrake, false);
    SmartMax intakeRight = new SmartMax(Constants.RIGHT_INTAKE_ID, IdleMode.kBrake, false);
    SmartMax intakeSpin = new SmartMax(Constants.SPIN_INTAKE_ID, IdleMode.kBrake, false);

    // CANSparkMax intakeLeft = new CANSparkMax(Constants.LEFT_INTAKE_ID , MotorType.kBrushless);
    // CANSparkMax intakeRight = new CANSparkMax(Constants.RIGHT_INTAKE_ID, MotorType.kBrushless);
    // CANSparkMax intakeSpin = new CANSparkMax(Constants.SPIN_INTAKE_ID, MotorType.kBrushless);

    PIDController controller = new PIDController(1.0, 0.0001, 0.05);
    DutyCycleEncoder enc = new DutyCycleEncoder(0);
    Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

  public IntakeSubsystem() {
    distanceSensor.setAutomaticMode(true);
    distanceSensor.setEnabled(true);
    distanceSensor.setRangeProfile(RangeProfile.kLongRange);
    controller.enableContinuousInput(0, 1);
  }

  public void resetSensor() {
    distanceSensor.setEnabled(false);
    distanceSensor.setEnabled(true);
  }

  public double getAbsolutePosition() {
    return enc.getAbsolutePosition();
  }

  public boolean getIsUp() {
    return Math.abs(IntakeConstants.RETRACTED_POS - getAbsolutePosition()) < .1;
  }

  public void setAnglePercentOutput(double point) {
    intakeLeft.set(point);
    intakeRight.set(-point);
  }

  public void setIntakeSpin(double point) {
    intakeSpin.set(point);
  }

  public void setPosition(double position) {
    double setpoint = controller.calculate(getAbsolutePosition(), position);
    setAnglePercentOutput(-setpoint);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  public double getProximity() {
    return distanceSensor.getRange();
  }

  public void checkSensor() {
    if(distanceSensor.GetRange() < 0) {
      distanceSensor.setEnabled(true);
      distanceSensor.setAutomaticMode(true);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Encoder" , enc.getAbsolutePosition());
    SmartDashboard.putNumber("Sensor Proximty", distanceSensor.getRange());
    // checkSensor();
    // controller.setP(SmartDashboard.getNumber("Intake angle P", 0));
    // SmartDashboard.putNumber("Current P", controller.getP());
    // goToAbsolutePos(currentGoalPos);
  }
}
