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

public class ShooterLineupCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSub;
  public ShooterLineupCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSub) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSub = shooterSub;
     
    addRequirements(shooterSub, intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeSpin(0);
  }

  @Override
  public void execute() {
    intakeSubsystem.setPosition(Constants.RETRACTED_POS);
    shooterSub.setPosition(Constants.SHOOTER_LINEUP_POSITION);

    SmartDashboard.putBoolean("AtSetpoint", shooterSub.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.setAngleSpeed(0);
    intakeSubsystem.setAnglePercentOutput(0);
  }

  @Override
  
  public boolean isFinished() {
    // return shooterSub.atSetpoint() && intakeSubsystem.atSetpoint();
    return false;
  }
}
