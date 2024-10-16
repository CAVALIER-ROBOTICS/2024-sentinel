// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpScoringCommand extends Command {
  /** Creates a new AmpScoringCommand. */
  ShooterSubsystem shooterSubsystem;
  DoubleSupplier flywheel, kicker;
  double angle = 1.05;
  double speed = .15;

  public AmpScoringCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier flywheel, DoubleSupplier kicker) {
    this.shooterSubsystem = shooterSubsystem;
    this.flywheel = flywheel;
    this.kicker = kicker;

    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // angle = SmartDashboard.getNumber(Constants.ShooterAngleAmpSD, 0);
    // speed = SmartDashboard.getNumber(Constants.FlywheelSpeedAmpSD, 0);

    shooterSubsystem.gotoAngle(angle, 0); //lmao
    shooterSubsystem.setKickerSpeed(-kicker.getAsDouble());
    shooterSubsystem.setFlywheelSpeed((int) (flywheel.getAsDouble() + .5) * speed, -.1);
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
