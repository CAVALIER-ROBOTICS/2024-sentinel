// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutonCommands.StartThetaOverrideCommand;
import frc.robot.commands.BotStateCommands.IntakeStateCommand;
import frc.robot.commands.BotStateCommands.SendbackCommand;
import frc.robot.commands.BotStateCommands.ShooterLineupCommand;
import frc.robot.commands.BotStateCommands.ShooterTransferCommand;
import frc.robot.commands.DriveCommands.FieldDrive;
import frc.robot.commands.ShooterCommands.AmpScoringCommand;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  ClimbSubsystem climb = new ClimbSubsystem();

  public RobotContainer() {
  
    NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
      new IntakeStateCommand(intake, shooterSubsystem),
      new RunCommand(() -> intake.setIntakeSpin(1), intake).withTimeout(.05),
      new ShooterLineupCommand(intake, shooterSubsystem).withTimeout(.5),
      new ShooterTransferCommand(intake, shooterSubsystem),
      new SendbackCommand(shooterSubsystem)
      // new FinishCommand(shooterSubsystem).raceWith(new WaitCommand(.02))
    ));
    // registerNamedCommands();
    PathLoader.configureAutoBuilder(driveSubsystem);
    SmartDashboard.putNumber(Constants.P_thetaSmartdashboard, 0);
    SmartDashboard.putNumber(Constants.I_thetaSmartdashboard, 0);
    SmartDashboard.putNumber(Constants.D_thetaSmartdashboard, 0);

    driveSubsystem.setDefaultCommand(new FieldDrive(

    driveSubsystem,

    () -> Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> driver.getRightX() * Math.PI));

    shooterSubsystem.setDefaultCommand(new ForceSendbackCommand(shooterSubsystem, operator::getRightTriggerAxis, operator::getLeftTriggerAxis));
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

  public Command getIntakeCommands() {
    return new SequentialCommandGroup(
            new IntakeStateCommand(intake, shooterSubsystem),
            new RunCommand(() -> intake.setIntakeSpin(1), intake).withTimeout(.05),
            new ShooterLineupCommand(intake, shooterSubsystem).withTimeout(.5),
            new ShooterTransferCommand(intake, shooterSubsystem),
            new SendbackCommand(shooterSubsystem));
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
    // JoystickButton runFlywheel = new JoystickButton(driver, 2);
    JoystickButton zeroGyro = new JoystickButton(driver, 4);
    JoystickButton targetTrack = new JoystickButton(driver, 2);
    JoystickButton ampMode = new JoystickButton(driver, 3);

    toggleIntake.onTrue(getIntakeCommands());
    
    zeroGyro.onTrue(new InstantCommand(driveSubsystem::resetGyroFieldDrive));
    ampMode.toggleOnTrue(new AmpScoringCommand(shooterSubsystem, operator::getRightTriggerAxis, operator::getLeftTriggerAxis));

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
    return PathLoader.loadPath(path);
  }

  public void RedRight() {

    SequentialCommandGroup Starting = new SequentialCommandGroup(
       intake(),
        new StartThetaOverrideCommand(shooterSubsystem), 
        PathLoader.loadPath("RedRightDynStarting")
    );
}

public SequentialCommandGroup intake() {

    return new SequentialCommandGroup(
      new IntakeStateCommand(intake, shooterSubsystem),
      new RunCommand(() -> intake.setIntakeSpin(1), intake).withTimeout(.05),
      new ShooterLineupCommand(intake, shooterSubsystem).withTimeout(.5),
      new ShooterTransferCommand(intake, shooterSubsystem),
      new SendbackCommand(shooterSubsystem)
      // new FinishCommand(shooterSubsystem).raceWith(new WaitCommand(.02))
    );
}
}
