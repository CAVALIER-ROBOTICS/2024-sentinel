// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.ClimbCommand;
import frc.robot.commands.AutonCommands.AngleShooterAndKickCommand;
import frc.robot.commands.AutonCommands.AngleShooterAndSpinupCommand;
import frc.robot.commands.AutonCommands.UltrashotAndSpinupCommand;
import frc.robot.commands.AutonCommands.StartThetaOverrideCommand;
import frc.robot.commands.AutonCommands.StopThetaOverrideCommand;
import frc.robot.commands.AutonCommands.UltrashotAndKickCommand;
import frc.robot.commands.BotStateCommands.IntakeStateCommand;
import frc.robot.commands.BotStateCommands.SendbackCommand;
import frc.robot.commands.BotStateCommands.ShooterFinishCommand;
import frc.robot.commands.BotStateCommands.ShooterLineupCommand;
import frc.robot.commands.BotStateCommands.ShooterTransferCommand;
import frc.robot.commands.DriveCommands.FieldDrive;
import frc.robot.commands.ShooterCommands.AmpScoringCommand;
import frc.robot.commands.ShooterCommands.ForceIntakeUpCommand;
import frc.robot.commands.ShooterCommands.ForceSendbackCommand;
import frc.robot.commands.ShooterCommands.UltrashotCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  ClimbSubsystem climb = new ClimbSubsystem();

  public void registerCommands() {
    NamedCommands.registerCommand("Intake", intake());
    NamedCommands.registerCommand("Shoot", getStationaryShotCommand());
  }

  public RobotContainer() {
    registerCommands();
    PathLoader.configureAutoBuilder(driveSubsystem);
    
    driveSubsystem.setDefaultCommand(new FieldDrive(

    driveSubsystem,

    () -> Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> driver.getRightX() * Math.PI));

    shooterSubsystem.setDefaultCommand(new ForceSendbackCommand(shooterSubsystem, operator::getRightBumper, operator::getLeftBumper));
    climb.setDefaultCommand(new ClimbCommand(climb, operator::getLeftY, operator::getRightY));
    configureBindings();
  }

  public Optional<Rotation2d> getRotationTargetOverride(){
    return shooterSubsystem.getRotationOverride();
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
  
  public Command startThetaOverrideCommand() {
    return new InstantCommand(() -> PPHolonomicDriveController.setRotationTargetOverride(
      () -> Optional.of(Rotation2d.fromRadians(shooterSubsystem.getAngleStates().getTheta()))
    ));
  }

  public Command endThetaOverrideCommand() {
  return new InstantCommand(() -> PPHolonomicDriveController.setRotationTargetOverride(
      () -> Optional.empty())
    );
  }

  private void configureBindings() {
    JoystickButton toggleIntake = new JoystickButton(driver, 5);
    JoystickButton zeroGyro = new JoystickButton(driver, 4);
    JoystickButton targetTrack = new JoystickButton(driver, 2);
    JoystickButton ampMode = new JoystickButton(driver, 3);
    JoystickButton retractIntake = new JoystickButton(operator, 1);

    toggleIntake.toggleOnTrue(intake());
    
    zeroGyro.onTrue(new InstantCommand(driveSubsystem::resetGyroFieldDrive));
    ampMode.toggleOnTrue(new AmpScoringCommand(shooterSubsystem, operator::getRightTriggerAxis, operator::getLeftTriggerAxis));

    retractIntake.whileTrue(new ForceIntakeUpCommand(intake));

    targetTrack.toggleOnTrue(new UltrashotCommand(shooterSubsystem, driveSubsystem, 
    () -> -Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> -Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

      operator::getLeftTriggerAxis,
      operator::getRightTriggerAxis));
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public Command getPathCommand(String path) {
    return PathLoader.loadAuto(path);
  }

//  public void RedRight() {
//     SequentialCommandGroup Starting = new SequentialCommandGroup(
//        intake(),
//         new StartThetaOverrideCommand(shooterSubsystem), 
//         PathLoader.loadPath("RedRightDynStarting")
//     );
// }

  public Command intake() {
      return new SequentialCommandGroup(
        new IntakeStateCommand(intake, shooterSubsystem),
        new RunCommand(() -> intake.setIntakeSpin(1), intake).withTimeout(.05),
        new ShooterLineupCommand(intake, shooterSubsystem).withTimeout(1),
        new ShooterTransferCommand(intake, shooterSubsystem),
        new ShooterFinishCommand(shooterSubsystem),
        new SendbackCommand(shooterSubsystem).withTimeout(.05) 
      ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command getShootingWhileMovingCommand() {
    return new SequentialCommandGroup(
      new StartThetaOverrideCommand(shooterSubsystem),
      new AngleShooterAndSpinupCommand(shooterSubsystem),
      new AngleShooterAndKickCommand(shooterSubsystem).withTimeout(1),
      new StopThetaOverrideCommand()
    );
  }

  public Command getStationaryShotCommand() {
    return new SequentialCommandGroup(
      new UltrashotAndSpinupCommand(shooterSubsystem, driveSubsystem).withTimeout(2),
      new UltrashotAndKickCommand(shooterSubsystem, driveSubsystem).withTimeout(2)
    );
  }
}
