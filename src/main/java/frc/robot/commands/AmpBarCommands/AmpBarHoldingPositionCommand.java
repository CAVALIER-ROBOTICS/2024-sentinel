// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpBarCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpBarSubsystem;

public class AmpBarHoldingPositionCommand extends Command {
  AmpBarSubsystem ampBarSubsystem;
  /** Creates a new AmpBarHoldingPositionCommand. */
  public AmpBarHoldingPositionCommand(AmpBarSubsystem ampBarSubsystem) {
    this.ampBarSubsystem = ampBarSubsystem;
    addRequirements(ampBarSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampBarSubsystem.setPosition(Constants.AmpBarConstants.AMPBAR_RETRACTED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampBarSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
