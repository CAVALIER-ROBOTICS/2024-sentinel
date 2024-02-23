// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ultrashot.AngleStates;

public class UltrashotAndKickCommand extends UltrashotAndSpinupCommand {
  public UltrashotAndKickCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    super(shooterSubsystem, driveSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.updateUltrashot(driveSubsystem);
    shooterSubsystem.ultimatum();
    shooterSubsystem.setFlywheelSpeed(Constants.ShooterConstants.MAX_FLYWHEEL_PERCENT_OUTPUT);
    shooterSubsystem.setKickerSpeed(-1);

    AngleStates states = shooterSubsystem.getAngleStates();

    driveSubsystem.driveWithAngleOverride(Rotation2d.fromRadians(states.getTheta()), 0, 0, states.getOmega()); // 0.1 is the heading controller D
    shooterSubsystem.gotoAngle(states.getPhi());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
