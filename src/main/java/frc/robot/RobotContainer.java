// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BotStateCommands.FinishCommand;
import frc.robot.commands.BotStateCommands.IntakeStateCommand;
import frc.robot.commands.BotStateCommands.SendbackCommand;
import frc.robot.commands.BotStateCommands.ShooterLineupCommand;
import frc.robot.commands.BotStateCommands.ShooterTransferCommand;
import frc.robot.commands.DriveCommands.FieldDrive;
import frc.robot.commands.ShooterCommands.UltrashotCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  ClimbSubsystem climb = new ClimbSubsystem();
  
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new FieldDrive(

    driveSubsystem,

    () -> Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> driver.getRightX() * Math.PI));

    // driveSubsystem.setDefaultCommand(new PrepUltrashotCommand(shooterSubsystem, driveSubsystem, driver::getLeftY, driver::getLeftX));
    
    configureBindings();
  }

  public int lol(boolean b) {
    if(b) {return 1;}
    return 0;
  }

  public double directionIsZero(double x, double y) {
    if (Math.abs(x) + Math.abs(y) < 0.1) {
      return 0.0;
    } else {
      return 1.0;
    }
  }

  private void configureBindings() {
    JoystickButton toggleIntake = new JoystickButton(driver, 1);
    // JoystickButton runFlywheel = new JoystickButton(driver, 2);
    JoystickButton zeroGyro = new JoystickButton(driver, 4);
    JoystickButton targetTrack = new JoystickButton(driver, 3);

    SequentialCommandGroup intakeSequence = new SequentialCommandGroup(
      new IntakeStateCommand(intake, shooterSubsystem),
      new RunCommand(() -> intake.setIntakeSpin(1), intake).raceWith(new WaitCommand(.05)),
      new ShooterLineupCommand(intake, shooterSubsystem).raceWith(new WaitCommand(1)),
      new ShooterTransferCommand(intake, shooterSubsystem),
      new SendbackCommand(shooterSubsystem),
      new FinishCommand(shooterSubsystem).raceWith(new WaitCommand(.025))
    );

    toggleIntake.onTrue(intakeSequence);
    zeroGyro.onTrue(new InstantCommand(driveSubsystem::resetGyroFieldDrive));
    targetTrack.whileTrue(new UltrashotCommand(shooterSubsystem, driveSubsystem, 
      () -> Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
      * directionIsZero(driver.getLeftX(), driver.getLeftY()),

      () -> Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
      * directionIsZero(driver.getLeftX(), driver.getLeftY()),

      driver::getLeftTriggerAxis));
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public Command getPathCommand(String path) {
    return PathLoader.loadPath(path);
  }
}
