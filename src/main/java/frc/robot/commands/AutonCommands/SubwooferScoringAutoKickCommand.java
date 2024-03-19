// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class SubwooferScoringAutoKickCommand extends SubwooferScoringAutoCommand {
  /** Creates a new SubwooferScoringAutoKickCommand. */
  public SubwooferScoringAutoKickCommand(ShooterSubsystem shooterSubsystem) {
    super(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setFlywheelSpeed(Constants.ShooterConstants.MAX_FLYWHEEL_PERCENT_OUTPUT);
    shooterSubsystem.gotoAngle(Math.toRadians(56), 0);
    shooterSubsystem.setKickerSpeed(-1);
  }
}
