// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class FieldDrive extends Command {
  DriveSubsystem dSub;
  DoubleSupplier x, y, rot;
  public FieldDrive(DriveSubsystem dSub, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    this.dSub = dSub;
    this.x = x;
    this.y = y;
    this.rot = rot;

    addRequirements(dSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      x.getAsDouble(), 
      y.getAsDouble(), 
      rot.getAsDouble(), 
      dSub.getAngle()
    );

    dSub.drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
