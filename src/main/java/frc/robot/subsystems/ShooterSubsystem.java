// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import java.util.Optional;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BasicLibrary.SmartMax;
import frc.robot.Constants.ShooterConstants;
import frc.robot.filters.BeamBreakFilter;
import frc.robot.filters.SimplerFilter;
import frc.robot.ultrashot.AngleStates;
import frc.robot.ultrashot.Point3D;
import frc.robot.ultrashot.UltraShot;
import frc.robot.ultrashot.UltraShot4;
import frc.robot.ultrashot.UltraShotConstants;
import frc.robot.vision.Limelight;

public class ShooterSubsystem extends SubsystemBase {

  SmartMax top = new SmartMax(Constants.TOP_SHOOTER_ID, IdleMode.kBrake, false);
  SmartMax bottom = new SmartMax(Constants.BOTTOM_SHOOTER_ID, IdleMode.kBrake, false);
  SmartMax left = new SmartMax(Constants.LEFT_SHOOTER_PIVOT_ID, IdleMode.kBrake, false);
  SmartMax right = new SmartMax(Constants.RIGHT_SHOOTER_PIVOT_ID, IdleMode.kBrake, false);
  SmartMax kicker = new SmartMax(Constants.KICKER_ID);

  BeamBreakFilter filter = new BeamBreakFilter();


  // CANSparkMax top = new CANSparkMax(Constants.TOP_SHOOTER_ID, MotorType.kBrushless);
  // CANSparkMax bottom = new CANSparkMax(Constants.BOTTOM_SHOOTER_ID, MotorType.kBrushless);
  // CANSparkMax left = new CANSparkMax(Constants.LEFT_SHOOTER_PIVOT_ID, MotorType.kBrushless);
  // CANSparkMax right = new CANSparkMax(Constants.RIGHT_SHOOTER_PIVOT_ID, MotorType.kBrushless);
  // CANSparkMax kicker = new CANSparkMax(Constants.KICKER_ID, MotorType.kBrushless);

  DigitalInput limit = new DigitalInput(ShooterConstants.SHOOTER_LIMIT_SWITCH_ID);

  DutyCycleEncoder enc = new DutyCycleEncoder(2);
  PIDController angleController = new PIDController(1.2, 0.0, 0.01);

  RelativeEncoder rpmEncoderTop, rpmEncoderBottom;
  UltraShot4 ultraShot = new UltraShot4();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    angleController.enableContinuousInput(0,2*Math.PI);
    rpmEncoderTop = top.getEncoder();
    rpmEncoderBottom = bottom.getEncoder();
    enc.reset();
  }

  public AngleStates getAngleStates() {
    return ultraShot.getAngleStates();
  }

  public void setFlywheelSpeed(double speed) {
    if(speed == 0) {
      top.set(0);
      bottom.set(0);
      return;
    }
    top.set(speed);
    bottom.set(-(speed + 0.025));
  }

   public void setFlywheelSpeed(double speed, double difference) {
    if(speed == 0) {
      top.set(0);
      bottom.set(0);
      return;
    }
    top.set(speed + difference + 0.075);
    bottom.set(-(speed - difference + 0.075));
  }
  
  public void stopAll() {
    setAngleSpeed(0);
    setFlywheelSpeed(0);
    setKickerSpeed(0);
  }

  public void setAngleSpeed(double angularSpeed) {
    SmartDashboard.putNumber("Requested shooter angular speed", angularSpeed);
    left.set(angularSpeed);
    right.set(-angularSpeed);
  }

  public void setKickerSpeed(double speed) {
    kicker.set(speed);
  }

  public double[] getRPM() {
    return new double[] {rpmEncoderTop.getVelocity(), rpmEncoderBottom.getVelocity()};
  }

  public double getAverageRPM() {
    double[] rpms = getRPM();
    return (rpms[0] + rpms[1]) / 2;
  }

  public double getAbsolutePosition() {
    return (enc.getAbsolutePosition() - Constants.ShooterConstants.SHOOTER_HORIZONTAL) * 2 * Math.PI; // Uncomment to find actual values
    // return (Constants.ShooterConstants.SHOOTER_HORIZONTAL - (enc.getAbsolutePosition())) *2*Math.PI;
  }

  public void setPosition(double position, double psi) {
    double setpoint = angleController.calculate(getAbsolutePosition(), position) + psi * angleController.getD();
    setAngleSpeed(setpoint);
  }

  public boolean atSetpoint() {
    return angleController.atSetpoint();
  }

  public boolean hasNoteInShooter() {
    return !SimplerFilter.filter(limit.get());
    // return !limit.get();
  }

  public double getNoteSpeed(double rpm) {
    return rpm * (1.0/60.0) * (Math.PI * 2) * (41.0/22.0) * 1.5 * .0254 * .587492749274927492; //last term is slipping constant
  }

  public void updateUltrashot(DriveSubsystem driveSubsystem) {
    ultraShot.update(driveSubsystem.getEstimatedPosition(), driveSubsystem.getChassisSpeeds(), getNoteSpeed(getAverageRPM()), 0.02);
  }

  private double clamp(double x, double min, double max) {
    return (x > max) ? max: (x < min) ? min: x;
  }

  public void pushMeasurementAndSetpoint(double setpoint) {
    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("measurement", getAbsolutePosition());
    SmartDashboard.putNumber("errornuts", getAbsolutePosition() - setpoint);
  }

  public void gotoAngle(double angle, double psi) {
    angle = clamp(angle, 0, (Math.PI / 2));
    pushMeasurementAndSetpoint(angle);
    setPosition(angle, psi);
  }

  // public void updatePID() {
  //   angleController.setP(SmartDashboard.getNumber(Constants.P_phiSmartdashboard, 0));
  //   angleController.setI(SmartDashboard.getNumber(Constants.I_phiSmartdashboard, 0));
  //   angleController.setD(SmartDashboard.getNumber(Constants.D_phiSmartdashboard, 0));
  // }

  public void updateTarget() {
    if(Limelight.targetBlue()) {
      ultraShot.setTargetSpeakerBlue();
      return;
    }
    ultraShot.setTargetSpeakerRed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AbsoluteShooterPosition", getAbsolutePosition());
    SmartDashboard.putNumber("RequestedShooterPosistion", ultraShot.getPhi());
    SmartDashboard.putNumber("FlywheelRPMTop", getRPM()[0]);
    SmartDashboard.putNumber("FlywheelRPMBottom", getRPM()[1]);
    SmartDashboard.putNumber("LeftCurrentDraw", left.getOutputCurrent());
    SmartDashboard.putNumber("RightCurrentDraw", right.getOutputCurrent());
    SmartDashboard.putBoolean("HasNote", hasNoteInShooter());

    updateTarget();
  }

  public Optional<Rotation2d> getRotationOverride() {
    return Optional.of(Rotation2d.fromRadians(ultraShot.getTheta()));
  }
}
