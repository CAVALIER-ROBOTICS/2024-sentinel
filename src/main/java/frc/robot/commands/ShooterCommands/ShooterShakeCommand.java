// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterShakeCommand extends Command {
  /** Creates a new ShooterShakeCommand. */
  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  double step = 0.0;

  public ShooterShakeCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    step += .1;
    double angle = (Math.PI / 2.5) * (Math.sin(step));
    angle = Math.abs(angle);
    shooterSubsystem.gotoAngle(angle, 0.0);

    intakeSubsystem.setPosition(IntakeConstants.EXTENDED_POS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAll();
    intakeSubsystem.setAnglePercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
