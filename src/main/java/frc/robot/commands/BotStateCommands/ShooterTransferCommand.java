// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BotStateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTransferCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;

  public ShooterTransferCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem);
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setIntakeSpin(1);
    shooterSubsystem.setKickerSpeed(-.5);
    shooterSubsystem.setPosition(Constants.SHOOTER_LINEUP_POSITION);
    intakeSubsystem.setPosition(Constants.RETRACTED_POS);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpin(0);
    intakeSubsystem.setAnglePercentOutput(0);
    shooterSubsystem.setKickerSpeed(0);
    shooterSubsystem.setAngleSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.hasNoteInShooter();
  }
}
