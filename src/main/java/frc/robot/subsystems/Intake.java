// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

    CANSparkMax intakeLeft = new CANSparkMax(Constants.LEFT_INTAKE_ID , MotorType.kBrushless);
    CANSparkMax intakeRight = new CANSparkMax(Constants.RIGHT_INTAKE_ID, MotorType.kBrushless);
    CANSparkMax intakeSpin = new CANSparkMax(Constants.SPIN_INTAKE_ID, MotorType.kBrushless);
    PIDController pid = new PIDController(0, 0, 0);
    DutyCycleEncoder enc = new DutyCycleEncoder(0);

    


  public Intake() {

    
    
    enc.setPositionOffset(0);

    
    intakeLeft.setInverted(true);


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Encoder" , enc.getAbsolutePosition());

  }
}
