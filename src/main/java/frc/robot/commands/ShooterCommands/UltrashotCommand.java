// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ultrashot.Point2D;

public class UltrashotCommand extends Command {
  /** Creates a new UltrashotCommand. */
  ShooterSubsystem shooterSubsystem;
  DriveSubsystem driveSubsystem;
  DoubleSupplier x, y;
  public UltrashotCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y) {
    this.shooterSubsystem = shooterSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    
    addRequirements(shooterSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    shooterSubsystem.updateUltrashot(driveSubsystem);
    Point2D shotConfig = shooterSubsystem.getUltraShotParameters();

    double shooterAngle = shotConfig.getX();
    double botAngle = shotConfig.getY();

    driveSubsystem.driveWithAngleOverride(Rotation2d.fromRadians(botAngle), x.getAsDouble(), y.getAsDouble());
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
