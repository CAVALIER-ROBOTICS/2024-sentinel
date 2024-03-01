// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands.StationaryShotCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ultrashot.AngleStates;
import frc.robot.ultrashot.UltraShotConstants;

public class UltrashotAndKickCommand extends UltrashotAndSpinupCommand {
  public UltrashotAndKickCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    super(shooterSubsystem, driveSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.updateUltrashot(driveSubsystem, UltraShotConstants.shooterSpeedAuto);
    shooterSubsystem.ultimatum();
    shooterSubsystem.setFlywheelSpeed(Constants.ShooterConstants.MAX_FLYWHEEL_PERCENT_OUTPUT);
    shooterSubsystem.setKickerSpeed(-1);

    AngleStates states = shooterSubsystem.getAngleStates();

    if(Double.isNaN(states.getTheta())) {return;}

    driveSubsystem.driveWithAngleOverride(Rotation2d.fromRadians(states.getTheta() + Math.PI), 0.001, 0.001, states.getOmega()); // 0.1 is the heading controller D
    shooterSubsystem.gotoAngle(states.getPhi(), states.getPsi());
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.hasNoteInShooter();
  }
}
