// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RobotDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new RobotDrive(driveSubsystem, driver::getLeftY, driver::getLeftX, driver::getRightX));
    configureBindings();

  }

  private void configureBindings() {
    JoystickButton toggleIntake = new JoystickButton(driver, 4);
    toggleIntake.toggleOnTrue(new IntakeCommand(intake));
  }

  public Command getAutonomousCommand() {
   return null;
  }
}
