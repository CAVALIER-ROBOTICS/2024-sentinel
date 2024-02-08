// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

    CANSparkMax intakeLeft = new CANSparkMax(Constants.LEFT_INTAKE_ID , MotorType.kBrushless);
    CANSparkMax intakeRight = new CANSparkMax(Constants.RIGHT_INTAKE_ID, MotorType.kBrushless);
    CANSparkMax intakeSpin = new CANSparkMax(Constants.SPIN_INTAKE_ID, MotorType.kBrushless);

    PIDController pid = new PIDController(0, 0, 0);
    DutyCycleEncoder enc = new DutyCycleEncoder(0);

    boolean isExtended = false;

  public IntakeSubsystem() {

    //enc.setPositionOffset(0);
    intakeLeft.setInverted(true);
    intakeRight.setInverted(false);
    intakeSpin.setIdleMode(IdleMode.kBrake);
    intakeRight.setIdleMode(IdleMode.kBrake);
    intakeLeft.setIdleMode(IdleMode.kBrake);

  }

  public void runIntake() {
    if (isExtended) {

      //stopIntake();
      isExtended = false;
    } else {
      
      //intakeSpin.set(0);
      isExtended = true;
    }
  }

  public void stopIntake() {

    intakeSpin.set(0);
    isExtended = false;
  }

  public void setPos() {

    if (isExtended) {

      intakeLeft.set(pid.calculate(enc.getAbsolutePosition(), Constants.extendedPos)); 
      intakeRight.set(pid.calculate(enc.getAbsolutePosition(), Constants.extendedPos));
      
    } else {

      intakeLeft.set(pid.calculate(enc.getAbsolutePosition(), Constants.retractedPos)); 
      intakeRight.set(pid.calculate(enc.getAbsolutePosition(), Constants.retractedPos));
    }

    if(isExtended) {

      intakeSpin.set(.5);
    } else {

      intakeSpin.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    setPos();
    SmartDashboard.putNumber("Intake Encoder" , enc.getAbsolutePosition());

  }
}
