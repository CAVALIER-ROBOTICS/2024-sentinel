// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpBarSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ultrashot.AngleStates;
import frc.robot.vision.Limelight;

public class RotateTowardsTarget extends Command {
  /** Creates a new UltrashotCommand. */
  ShooterSubsystem shooterSubsystem;
  DriveSubsystem driveSubsystem;
  DoubleSupplier x, y;

  public RotateTowardsTarget(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y) {
    this.shooterSubsystem = shooterSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    
    addRequirements(shooterSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetHeadingPID();
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    shooterSubsystem.updateUltrashot(driveSubsystem);

    AngleStates states = shooterSubsystem.getAngleStates();

    if(Double.isNaN(states.getTheta())) {return;}

    driveSubsystem.driveWithAngleOverride(Rotation2d.fromRadians(states.getTheta() + Math.PI), x.getAsDouble(), y.getAsDouble(), states.getOmega()); // 0.1 is the heading controller D

    SmartDashboard.putNumber("theta", states.getTheta());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Targeting", false);
    driveSubsystem.drive(new ChassisSpeeds());
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Limelight.getDistanceToTargetTag() > 0;
  }
}
