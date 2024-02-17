  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinCommand extends Command {
  /** Creates a new SpinCommand. */
  IntakeSubsystem intakeSub;
  double initialColorSensorValue;
  public SpinCommand(IntakeSubsystem isub) {
    this.intakeSub = isub;
    addRequirements(isub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialColorSensorValue = intakeSub.getProximity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setIntakeSpin(1);
    // SmartDashboard.putNumber("Offset", getOffsetFromInitialProximity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setIntakeSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSub.getIsUp() || intakeSub.getProximity() > Constants.MINIMUM_PROXIMITY_TRIGGER;
  }
}
