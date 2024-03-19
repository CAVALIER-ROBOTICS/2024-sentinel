// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpBarCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpBarSubsystem;

public class RetractAmpBarCommand extends ExtendAmpBarCommand {
  /** Creates a new RetractAmpBarCommand. */
  public RetractAmpBarCommand(AmpBarSubsystem asub) {
    super(asub);
  }

  @Override
  public void execute() {
    // System.out.println("Trying 2 retract");
    asub.setPosition(Constants.AmpBarConstants.AMPBAR_RETRACTED);
  }
}
