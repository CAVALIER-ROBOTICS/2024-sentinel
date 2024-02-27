// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class VectorFieldCommand extends Command {
  /** Creates a new VectorFieldCommand. */
  DriveSubsystem dSub;
  DoubleSupplier x, y, rot;
  public VectorFieldCommand(DriveSubsystem dSub, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    this.dSub = dSub;
    this.x = x;
    this.y = y;
    this.rot = rot;

    addRequirements(dSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
      x.getAsDouble(), 
      y.getAsDouble(), 
      rot.getAsDouble(), 
      dSub.getFieldDriveAngle()
    );

    dSub.drive(speeds);
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
