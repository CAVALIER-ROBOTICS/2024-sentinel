// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ForceSendbackCommand extends Command {
  /** Creates a new ForceSendbackCommand. */
  ShooterSubsystem shooterSubsystem;
  BooleanSupplier kF, kB;
  public ForceSendbackCommand(ShooterSubsystem shooterSubsystem, BooleanSupplier kF, BooleanSupplier kB) {
    this.shooterSubsystem = shooterSubsystem;
    this.kF = kF;
    this.kB = kB;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(kF.getAsBoolean()) {
      shooterSubsystem.setKickerSpeed(-1);
      return;
    }
    if(kB.getAsBoolean()) {
      shooterSubsystem.setKickerSpeed(1);
      return;
    }
    shooterSubsystem.setKickerSpeed(0);
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
