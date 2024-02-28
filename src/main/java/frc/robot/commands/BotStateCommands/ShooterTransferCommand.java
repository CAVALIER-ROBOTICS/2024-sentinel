// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BotStateCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
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
  public void initialize() {SmartDashboard.putBoolean("hasNote", false);}

  @Override
  public void execute() {
    intakeSubsystem.setIntakeSpin(1);
    shooterSubsystem.setKickerSpeed(-0.9);
    shooterSubsystem.setPosition(ShooterConstants.SHOOTER_LINEUP_POSITION, 0);
    intakeSubsystem.setPosition(IntakeConstants.RETRACTED_POS);
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
