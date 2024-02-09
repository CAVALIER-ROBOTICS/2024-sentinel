// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SpinCommand;
import frc.robot.commands.BotStateCommands.IntakeStateCommand;
import frc.robot.commands.BotStateCommands.ShooterLineupCommand;
import frc.robot.commands.BotStateCommands.ShooterTransferCommand;
import frc.robot.commands.DriveCommands.GarrettDrive;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new GarrettDrive(driveSubsystem, driver::getLeftY, driver::getLeftX, driver::getRightTriggerAxis, driver::getRightX));
    climbSubsystem.setDefaultCommand(new ClimbCommand(climbSubsystem, operator::getRightY));
    
    configureBindings();
  }

  private void configureBindings() {
    JoystickButton toggleIntake = new JoystickButton(driver, 1);
    // toggleIntake.onTrue(new IntakeCommand(intake).andThen(new SpinCommand(intake)));

    SequentialCommandGroup intakeSequence = new SequentialCommandGroup(
      new IntakeStateCommand(intake),
      new ShooterLineupCommand(intake, shooterSubsystem),
      new ShooterTransferCommand(intake, shooterSubsystem)
    );

    toggleIntake.onTrue(intakeSequence);
  }

  public Command getAutonomousCommand() {
   return null;
  }
}
