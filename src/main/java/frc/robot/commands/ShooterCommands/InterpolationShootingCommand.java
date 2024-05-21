// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Interpolation.InterpolatingTable;
import frc.robot.Interpolation.ShotParam;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.Limelight;

public class InterpolationShootingCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  DriveSubsystem driveSubsystem;
  DoubleSupplier x, y, kicker;
  /** Creates a new InterpolationShootingCommand. */
  public InterpolationShootingCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier kicker) {
    this.shooterSubsystem = shooterSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.kicker = kicker;
    addRequirements(shooterSubsystem, driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = Limelight.getDistanceToTargetTag();
    ShotParam param = InterpolatingTable.getShotParameter(distance);

    shooterSubsystem.shootFromShotParameter(param);
    driveSubsystem.driveWithApriltagCentering(x.getAsDouble(), y.getAsDouble());

    shooterSubsystem.setKickerSpeed(-kicker.getAsDouble());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
