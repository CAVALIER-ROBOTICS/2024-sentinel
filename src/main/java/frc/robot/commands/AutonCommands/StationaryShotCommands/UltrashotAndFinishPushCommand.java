// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands.StationaryShotCommands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class UltrashotAndFinishPushCommand extends UltrashotAndKickCommand {
  /** Creates a new UltrashotAndFinishCommand. */
  public UltrashotAndFinishPushCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    super(shooterSubsystem, driveSubsystem);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
