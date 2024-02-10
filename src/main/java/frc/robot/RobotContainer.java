// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BotStateCommands.IntakeStateCommand;
import frc.robot.commands.BotStateCommands.ShooterLineupCommand;
import frc.robot.commands.BotStateCommands.ShooterTransferCommand;
import frc.robot.commands.DriveCommands.FieldDrive;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  
  public RobotContainer() {
    PathLoader.configureAutoBuilder(driveSubsystem);

    driveSubsystem.setDefaultCommand(new FieldDrive(

        driveSubsystem,

        () -> Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
            * directionIsZero(driver.getLeftX(), driver.getLeftY()),

        () -> Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
            * directionIsZero(driver.getLeftX(), driver.getLeftY()),
   
        () -> driver.getRightX() * 2 * Math.PI));

    
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

  public double directionIsZero(double x, double y) {
    if (Math.abs(x) + Math.abs(y) < 0.1) {
      return 0.0;
    } else {
      return 1.0;
    }
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public Command getPathCommand(String path) {
    return PathLoader.loadPath(path);
  }
}
