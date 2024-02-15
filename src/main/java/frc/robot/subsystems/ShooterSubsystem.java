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
import frc.robot.ultrashot.AngleStates;
import frc.robot.ultrashot.UltraShot;
import frc.robot.ultrashot.UltraShotConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax top = new CANSparkMax(Constants.TOP_SHOOTER_ID, MotorType.kBrushless);
  CANSparkMax bottom = new CANSparkMax(Constants.BOTTOM_SHOOTER_ID, MotorType.kBrushless);
  CANSparkMax left = new CANSparkMax(Constants.LEFT_SHOOTER_PIVOT_ID, MotorType.kBrushless);
  CANSparkMax right = new CANSparkMax(Constants.RIGHT_SHOOTER_PIVOT_ID, MotorType.kBrushless);
  CANSparkMax kicker = new CANSparkMax(Constants.KICKER_ID, MotorType.kBrushless);

  DigitalInput limit = new DigitalInput(Constants.SHOOTER_LIMIT_SWITCH_ID);

  DutyCycleEncoder enc = new DutyCycleEncoder(1);
  PIDController angleController = new PIDController(0.5, 0.00, 0.0);

  RelativeEncoder rpmEncoderTop, rpmEncoderBottom;
  UltraShot ultraShot = new UltraShot();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    angleController.enableContinuousInput(0,2*Math.PI);

    // enc.setPositionOffset(0.08);
    // enc.setDistancePerRotation(2*Math.PI);

    top.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    bottom.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);
    top.setIdleMode(IdleMode.kCoast);
    bottom.setIdleMode(IdleMode.kCoast);

    rpmEncoderTop = top.getEncoder();
    rpmEncoderBottom = bottom.getEncoder();

    configUltrashot();

    enc.reset();
  }

  public void configUltrashot() {
    ultraShot.configure(
      UltraShotConstants.robot,
      UltraShotConstants.axis,
      UltraShotConstants.velocity,
      UltraShotConstants.target,
      UltraShotConstants.states,
      UltraShotConstants.shooterLength,
      UltraShotConstants.shooterSpeed,
      UltraShotConstants.localGravity,
      UltraShotConstants.airDrag,
      UltraShotConstants.settleTime
    );

  }
  public void setFlywheelSpeed(double speed) {
    top.set(speed);
    bottom.set(-(speed));
  }

  public void setAngleSpeed(double speed) {
    SmartDashboard.putNumber("Requested shooter speed", speed);
    left.set(speed);
    right.set(-speed);
  }

  public void setKickerSpeed(double speed) {
    kicker.set(speed);
  }

  public double getRPM() {
    return (rpmEncoderTop.getVelocity() + rpmEncoderBottom.getVelocity());
  }

  public double getAbsolutePosition() {
    return ((1-((enc.getAbsolutePosition()-0.09)))%1) *2*Math.PI;
  }

  public void setPosition(double position) {
    double setpoint = angleController.calculate(getAbsolutePosition(), position);
    setAngleSpeed(setpoint);
  }

  public boolean atSetpoint() {
    return angleController.atSetpoint();
  }

  public boolean hasNoteInShooter() {
    return limit.get();
  }

  // public void updateUltrashot(ChassisSpeeds botVelocity, Pose2d pose) {
  //   ultraShot.setRobotPos(pose);
  //   ultraShot.setRobotVelocity(botVelocity);
  // }

  public void updateUltrashot(DriveSubsystem driveSubsystem) {
    ultraShot.update(driveSubsystem.getOdometry(), driveSubsystem.getChassisSpeeds());
  }

  public void gotoAngle(double angle) {
    SmartDashboard.putNumber("Desired encoder rot", angle);
    SmartDashboard.putNumber("Current encoder rot", getAbsolutePosition());
    setPosition(angle);
  }

  public void track() {
    ultraShot.track();
  }
  
  public void ultimatum() {
    ultraShot.ultimatum();
  }
  public AngleStates getAngleStates() {
    return ultraShot.getAngleStates();
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
    SmartDashboard.putNumber("CalculatedAbsoluteShooterPosition", getAbsolutePosition());
    SmartDashboard.putNumber("FlywheelRPM", getRPM());
    SmartDashboard.putNumber("LeftCurrentDraw", left.getOutputCurrent());
    SmartDashboard.putNumber("RightCurrentDraw", right.getOutputCurrent());
    // This method will be called once per scheduler run
    configUltrashot();
  }
}
