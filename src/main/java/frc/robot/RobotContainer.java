// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AmpBarCommands.AmpBarHoldingPositionCommand;
import frc.robot.commands.AmpBarCommands.ExtendAmpBarCommand;
import frc.robot.commands.AmpBarCommands.RetractAmpBarCommand;
import frc.robot.commands.AutonCommands.AngleShooterAndKickCommand;
import frc.robot.commands.AutonCommands.AngleShooterAndSpinupCommand;
import frc.robot.commands.AutonCommands.IdleShooterSpin;
import frc.robot.commands.AutonCommands.VectorFieldCommand;
import frc.robot.commands.AutonCommands.StationaryShotCommands.UltrashotAndFinishKickCommand;
import frc.robot.commands.AutonCommands.StationaryShotCommands.UltrashotAndFinishPushCommand;
import frc.robot.commands.AutonCommands.StationaryShotCommands.UltrashotAndKickCommand;
import frc.robot.commands.AutonCommands.StationaryShotCommands.UltrashotAndSpinupCommand;
import frc.robot.commands.AutonCommands.StartThetaOverrideCommand;
import frc.robot.commands.AutonCommands.StopThetaOverrideCommand;
import frc.robot.commands.AutonCommands.SubwooferScoringAutoCommand;
import frc.robot.commands.AutonCommands.SubwooferScoringAutoKickCommand;
import frc.robot.commands.BotStateCommands.IntakeStateCommand;
import frc.robot.commands.BotStateCommands.ReverseIntakeCommand;
import frc.robot.commands.BotStateCommands.SendbackCommand;
import frc.robot.commands.BotStateCommands.ShooterFinishCommand;
import frc.robot.commands.BotStateCommands.ShooterLineupCommand;
import frc.robot.commands.BotStateCommands.ShooterTransferCommand;
import frc.robot.commands.DriveCommands.FieldDrive;
import frc.robot.commands.ShooterCommands.AmpScoringCommand;
import frc.robot.commands.ShooterCommands.ForceIntakeUpCommand;
import frc.robot.commands.ShooterCommands.ForceSendbackCommand;
import frc.robot.commands.ShooterCommands.SubwooferScoringCommand;
import frc.robot.commands.ShooterCommands.TeammatePassCommand;
import frc.robot.commands.ShooterCommands.UltrashotCommand;
import frc.robot.commands.ShooterCommands.ShooterIntakeCommands.IndexNoteInShooterCommand;
import frc.robot.commands.ShooterCommands.ShooterIntakeCommands.ShooterIntakeCommand;
import frc.robot.subsystems.AmpBarSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vectorfields.VectorFieldGenerator;
import frc.robot.vision.Limelight;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  // ClimbSubsystem climb = new ClimbSubsystem();
  AmpBarSubsystem ampBarSubsystem = new AmpBarSubsystem();

  public void registerCommands() {
    NamedCommands.registerCommand("Intake", intake());
    NamedCommands.registerCommand("ShootStation", getStationaryShotCommandWithTimers());
    NamedCommands.registerCommand("ShootMoving", getShootingWhileMovingCommand());
    NamedCommands.registerCommand("ShooterSpin", new IdleShooterSpin(shooterSubsystem));
    NamedCommands.registerCommand("DisableRamp", new InstantCommand(() -> driveSubsystem.setDriveMotorRampRate(0)));
    NamedCommands.registerCommand("EnableRamp", new InstantCommand(() -> Commands.none()));
    NamedCommands.registerCommand("SetpointShot", getSetpointShotCommand());
  }

  public Command getUltrashotDrivingCommand() {
      return new UltrashotCommand(
      shooterSubsystem, driveSubsystem, ampBarSubsystem,
      () -> -Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

      () -> -Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100)
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

      operator::getLeftTriggerAxis
      );
  }
    

  public RobotContainer() {
    SmartDashboard.putNumber(Constants.P_thetaSmartdashboard, 0);
    SmartDashboard.putNumber(Constants.I_thetaSmartdashboard, 0);
    SmartDashboard.putNumber(Constants.D_thetaSmartdashboard, 0);

    registerCommands();
    SmartDashboard.putNumber("ShooterSpeedEntry", 0);
    PathLoader.configureAutoBuilder(driveSubsystem);
    // PiHandler.initialize();
    driveSubsystem.setDriveMotorRampRate(0);
    
    driveSubsystem.setDefaultCommand(new FieldDrive(

    driveSubsystem,

    () -> Math.sin(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100) // 420 / 100
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> Math.cos(Math.atan2(driver.getLeftY(), driver.getLeftX())) * driver.getRightTriggerAxis() * (420 / 100) // 420 / 100
        * directionIsZero(driver.getLeftX(), driver.getLeftY()),

    () -> driver.getRightX() * Math.PI));

    shooterSubsystem.setDefaultCommand(new ForceSendbackCommand(shooterSubsystem, operator::getRightBumper, operator::getLeftBumper));
    ampBarSubsystem.setDefaultCommand(new AmpBarHoldingPositionCommand(ampBarSubsystem));
    // climb.setDefaultCommand(new ClimbCommand(climb, operator::getLeftY, operator::getRightY));
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
    JoystickButton targetTrack = new JoystickButton(operator, 2);
    JoystickButton ampMode = new JoystickButton(driver, 3);
    JoystickButton subwooferMode = new JoystickButton(operator, 4);
    JoystickButton retractIntake = new JoystickButton(operator, 1);
    JoystickButton forceOutIntake = new JoystickButton(operator, 3);
    // JoystickButton teammatePass = new JoystickButton(operator, 8);

    toggleIntake.toggleOnTrue(intake());
    
    zeroGyro.onTrue(new InstantCommand(driveSubsystem::resetGyroFieldDrive));

    ampMode.toggleOnTrue(getAmpScoringCommand(operator::getRightTriggerAxis, operator::getLeftTriggerAxis));
    ampMode.toggleOnFalse(new RetractAmpBarCommand(ampBarSubsystem));

    subwooferMode.toggleOnTrue(new SubwooferScoringCommand(shooterSubsystem, operator::getLeftTriggerAxis));

    retractIntake.whileTrue(new ForceIntakeUpCommand(intake));
    forceOutIntake.whileTrue(new ReverseIntakeCommand(intake));

    targetTrack.whileTrue(getUltrashotDrivingCommand());
    // teammatePass.toggleOnTrue(new TeammatePassCommand(shooterSubsystem, operator::getRightTriggerAxis, operator::getLeftTriggerAxis));
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;

  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public Command getPathCommand(String path) {
    return PathLoader.loadAuto(path);
  }

  public Command getSetpointShotCommand() {
    return new SequentialCommandGroup(
      new SubwooferScoringAutoCommand(shooterSubsystem).withTimeout(1.5),
      new SubwooferScoringAutoKickCommand(shooterSubsystem).withTimeout(1)
    );
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

  public Command getAmpScoringCommand(DoubleSupplier flywheel, DoubleSupplier kicker) {
    return new ParallelCommandGroup(new AmpScoringCommand(shooterSubsystem, flywheel, kicker), new ExtendAmpBarCommand(ampBarSubsystem)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
      new UltrashotAndSpinupCommand(shooterSubsystem, driveSubsystem).withTimeout(1),
      new UltrashotAndKickCommand(shooterSubsystem, driveSubsystem),
      new UltrashotAndFinishKickCommand(shooterSubsystem, driveSubsystem),
      new UltrashotAndFinishPushCommand(shooterSubsystem, driveSubsystem).withTimeout(.05)
    );
  }

  public Command getStationaryShotCommandWithTimers() {
    return new SequentialCommandGroup(
      new UltrashotAndSpinupCommand(shooterSubsystem, driveSubsystem).withTimeout(1),
      new UltrashotAndFinishPushCommand(shooterSubsystem, driveSubsystem).withTimeout(2)
    );
  }

  public Command getVectorFieldCommand() {
    VectorFieldGenerator vectorField = driveSubsystem.getVectorFieldGenerator();
    return new VectorFieldCommand(
      driveSubsystem,
      () -> vectorField.getVelocity().getX(),
      () -> vectorField.getVelocity().getY(),
      () -> 0
    );
  }

  public Command shooterIntakeCommand() {
    return new SequentialCommandGroup(
      new ShooterIntakeCommand(shooterSubsystem),
      new IndexNoteInShooterCommand(shooterSubsystem)
    );
  }
}
