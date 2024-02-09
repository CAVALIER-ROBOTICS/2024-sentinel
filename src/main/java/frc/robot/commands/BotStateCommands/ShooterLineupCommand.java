// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BotStateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLineupCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSub;
  public ShooterLineupCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSub) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSub = shooterSub;
     
    addRequirements(shooterSub);
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // shooterSub.setPosition(Constants.SHOOTER_LINEUP_POSITION);
    intakeSubsystem.setPosition(Constants.RETRACTED_POS);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.setAngleSpeed(0);
    intakeSubsystem.setAnglePercentOutput(0);
  }

  @Override
  
  public boolean isFinished() {
    return false;
    // return shooterSub.atSetpoint() && intakeSubsystem.atSetpoint();
  }
}
