// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.ultrashot.Point2D;

public class RiptideCommand extends Command {
  DriveSubsystem driveSubsystem;
  /** Creates a new RiptideCommand. */
  public RiptideCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.updateRiptide();
    Point2D speeds = driveSubsystem.getRiptideSpeeds();
    ChassisSpeeds raw = new ChassisSpeeds(speeds.getX(), speeds.getY(), 0);
    ChassisSpeeds speedsGood = ChassisSpeeds.fromFieldRelativeSpeeds(raw, driveSubsystem.getAngle());
    driveSubsystem.drive(speedsGood);
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
