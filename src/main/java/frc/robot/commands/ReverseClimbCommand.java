// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimbSubsystem;

public class ReverseClimbCommand extends Command {
  /** Creates a new StopClimbCommand. */
  ClimbSubsystem sub;
  boolean isInverted;
  
  public ReverseClimbCommand(ClimbSubsystem sub, boolean isInverted) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sub = sub;
    this.isInverted = isInverted;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(isInverted)
    {
      sub.setInverted();
    }
    else
    {
      sub.setNormal();
    }
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
