// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunFlywheelCommand extends Command {
  /** Creates a new RunFlywheelCommand. */
  ShooterSubsystem sub;
  public RunFlywheelCommand(ShooterSubsystem sub) {
    this.sub = sub;
    addRequirements(sub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sub.setFlywheelSpeed(.5);

    if(sub.getRPM() > 2500) {
      sub.setKickerSpeed(-1);
    } else {
      sub.setKickerSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub.setFlywheelSpeed(0);
    sub.setKickerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("RPM", sub.getRPM());
    return false;
  }
}
