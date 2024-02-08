// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RobotDrive extends Command {
  /** Creates a new RobotDrive. */
  DriveSubsystem dsub;
  DoubleSupplier x, y, rot;
  public RobotDrive(DriveSubsystem dsub, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    this.x = x;
    this.y = y;
    this.rot = rot;
    this.dsub = dsub;

    addRequirements(dsub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dsub.drive(new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
