// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AngleAdjustCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  DoubleSupplier angleSub;
  DoubleSupplier kickerSub;

  /** Creates a new AngleAdjustCommand. */
  public AngleAdjustCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier angleSub, DoubleSupplier kickerSub) {
    this.shooterSubsystem = shooterSubsystem;
    this.angleSub = angleSub;
    this.kickerSub = kickerSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelSpeed(.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setFlywheelSpeed(.25);
    shooterSubsystem.setAngleSpeed(-0.25 * angleSub.getAsDouble());
    shooterSubsystem.setKickerSpeed(-kickerSub.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setAngleSpeed(0);
    shooterSubsystem.setKickerSpeed(0);
    shooterSubsystem.setFlywheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
