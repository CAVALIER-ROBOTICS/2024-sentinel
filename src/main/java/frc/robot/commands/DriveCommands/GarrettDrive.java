// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class GarrettDrive extends Command {
  DriveSubsystem driveSubsystem;

  DoubleSupplier leftX, leftY, throttle, rightX;

  private int directionIsZero(double x, double y) {
    if (Math.abs(x) + Math.abs(y) < 0.1) {
      return 0;
    } else {
      return 1;
    }
  }

  private double[] getXandYInput() {
    double[] values = new double[2];
    double lmao = (Math.atan2(leftY.getAsDouble(), leftX.getAsDouble())) * throttle.getAsDouble() * (420 / 100) * directionIsZero(leftX.getAsDouble(), leftY.getAsDouble());
    
    values[0] = Math.sin(lmao);
    values[1] = -Math.cos(lmao);

    return values;
  }

  public GarrettDrive(DriveSubsystem dSub, DoubleSupplier x, DoubleSupplier y, DoubleSupplier throttle, DoubleSupplier rX) {
    this.driveSubsystem = dSub;
    this.leftX = x;
    this.leftY = y;
    this.rightX = rX;
    this.throttle = throttle;

    addRequirements(dSub);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d mod = Rotation2d.fromDegrees(-driveSubsystem.getAngle().getDegrees() % 360);
    double[] xy = getXandYInput();

    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xy[0],
        xy[1],
        rightX.getAsDouble() * Math.PI * 2,
        mod);

    ChassisSpeeds speeds = fieldSpeeds;

    driveSubsystem.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}