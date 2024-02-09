// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem isub;
  boolean isUp;
  double set;
  public IntakeCommand(IntakeSubsystem isub) {
    this.isub = isub;
    addRequirements(isub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     isUp = isub.getIsUp();
     set = (isUp == true) ? .1: -.1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isub.setAnglePercentOutput(set);
    SmartDashboard.putBoolean("IsUp", isub.getIsUp());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isub.setAnglePercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double goalPos = (isUp) ? Constants.EXTENDED_POS: Constants.RETRACTED_POS;
    SmartDashboard.putNumber("ShooterError", Math.abs(isub.getAbsolutePosition() - goalPos));
    if((Math.abs(goalPos - isub.getAbsolutePosition()) < .1)) {
      return true;
    }
    return false;
  }
}
