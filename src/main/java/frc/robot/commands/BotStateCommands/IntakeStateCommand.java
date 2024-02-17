// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BotStateCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeStateCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  public IntakeStateCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {SmartDashboard.putBoolean("hasNote", false);}

  @Override
  public void execute() {
    intakeSubsystem.setPosition(Constants.IntakeConstants.EXTENDED_POS);
    shooterSubsystem.setPosition(Constants.IntakeConstants.RETRACTED_POS);
    intakeSubsystem.setIntakeSpin(1);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpin(0);
    intakeSubsystem.setAnglePercentOutput(0);
    shooterSubsystem.setAngleSpeed(0);
    SmartDashboard.putBoolean("hasNote", true);
  }

  @Override
  public boolean isFinished() {
    //We have a note in the intake, now move it to the shooter.
    return intakeSubsystem.getProximity() <= Constants.IntakeConstants.MINIMUM_PROXIMITY_TRIGGER;
  }
}
