// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ultrashot.AngleStates;

public class AngleShooterAndKickCommand extends Command {
  /** Creates a new AngleShooterAndShootCommand. */
  ShooterSubsystem shooterSubsystem;
  public AngleShooterAndKickCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AngleStates states = shooterSubsystem.getAngleStates();

    shooterSubsystem.setFlywheelSpeed(Constants.ShooterConstants.MAX_FLYWHEEL_PERCENT_OUTPUT);
    shooterSubsystem.gotoAngle(states.getPhi(), states.getPsi());
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
