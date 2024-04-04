// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class SubwooferScoringCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  DoubleSupplier kick;
  /** Creates a new SubwooferScoringCommand. */
  public SubwooferScoringCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier kick) {
    this.shooterSubsystem = shooterSubsystem;
    this.kick = kick;

    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setFlywheelSpeed(Constants.ShooterConstants.MAX_FLYWHEEL_PERCENT_OUTPUT);
    shooterSubsystem.setKickerSpeed(-kick.getAsDouble());
    shooterSubsystem.gotoAngle(Math.toRadians(63), 0);
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
