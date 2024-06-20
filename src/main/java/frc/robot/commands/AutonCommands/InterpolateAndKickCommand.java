// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Interpolation.InterpolatingTable;
import frc.robot.Interpolation.ShotParam;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.Limelight;

public class InterpolateAndKickCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  DriveSubsystem driveSubsystem;
  /** Creates a new InterpolateAndKickCommand. */
  public InterpolateAndKickCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    addRequirements(shooterSubsystem);
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
    driveSubsystem.driveWithApriltagCentering(0, 0);

    shooterSubsystem.setKickerSpeed(-1);
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
