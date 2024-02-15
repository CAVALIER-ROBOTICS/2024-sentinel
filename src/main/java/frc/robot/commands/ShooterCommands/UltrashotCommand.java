// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ultrashot.AngleStates;

public class UltrashotCommand extends Command {
  /** Creates a new UltrashotCommand. */
  ShooterSubsystem shooterSubsystem;
  DriveSubsystem driveSubsystem;
  DoubleSupplier x, y, k, f;
  public UltrashotCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier k, DoubleSupplier f) {
    this.shooterSubsystem = shooterSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.k = k;
    this.f = f;
    addRequirements(shooterSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    shooterSubsystem.updateUltrashot(driveSubsystem);
    shooterSubsystem.ultimatum();
    shooterSubsystem.setFlywheelSpeed( (int) (f.getAsDouble() + .5) * .6);

    AngleStates states = shooterSubsystem.getAngleStates();

    driveSubsystem.driveWithAngleOverride(Rotation2d.fromRadians(states.getTheta()), x.getAsDouble(), y.getAsDouble());
    shooterSubsystem.gotoAngle(states.getPhi());
    shooterSubsystem.setKickerSpeed(-k.getAsDouble());
    SmartDashboard.putNumber("theta", states.getTheta());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelSpeed(0);
    shooterSubsystem.setAngleSpeed(0);
    shooterSubsystem.setKickerSpeed(0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
