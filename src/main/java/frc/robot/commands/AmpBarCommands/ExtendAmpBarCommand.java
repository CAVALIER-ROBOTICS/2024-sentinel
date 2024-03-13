// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpBarCommands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpBarSubsystem;

public class ExtendAmpBarCommand extends Command {
  AmpBarSubsystem asub;
  /** Creates a new ExtendAmpBarCommand. */
  public ExtendAmpBarCommand(AmpBarSubsystem asub) {
    this.asub = asub;
    addRequirements(asub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    asub.setPosition(Constants.AmpBarConstants.AMPBAR_EXTENDED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
