// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AmpBarCommands.AmpBarHoldingPositionCommand;
import frc.robot.commands.AmpBarCommands.ExtendAmpBarCommand;
import frc.robot.commands.AmpBarCommands.RetractAmpBarCommand;
import frc.robot.commands.AutonCommands.IdleShooterSpin;
import frc.robot.commands.AutonCommands.InterpolateAndKickCommand;
import frc.robot.commands.AutonCommands.InterpolateAndSpinupCommand;
import frc.robot.commands.AutonCommands.SubwooferScoringAutoCommand;
import frc.robot.commands.AutonCommands.SubwooferScoringAutoKickCommand;
import frc.robot.commands.AutonCommands.StationaryShotCommands.UltrashotAndKickCommand;
import frc.robot.commands.AutonCommands.StationaryShotCommands.UltrashotAndSpinupCommand;
import frc.robot.commands.BotStateCommands.IntakeStateCommand;
import frc.robot.commands.BotStateCommands.ReverseIntakeCommand;
import frc.robot.commands.BotStateCommands.ShooterFinishCommand;
import frc.robot.commands.BotStateCommands.ShooterLineupCommand;
import frc.robot.commands.BotStateCommands.ShooterTransferCommand;
import frc.robot.commands.DriveCommands.FieldDrive;
// import frc.robot.commands.ManualShooterCommands.AngleAdjustCommand;
import frc.robot.commands.ShooterCommands.AmpScoringCommand;
import frc.robot.commands.ShooterCommands.ForceIntakeUpCommand;
import frc.robot.commands.ShooterCommands.ForceSendbackCommand;
import frc.robot.commands.ShooterCommands.InterpolationShootingCommand;
import frc.robot.commands.ShooterCommands.RotateTowardsTarget;
import frc.robot.commands.ShooterCommands.SubwooferScoringCommand;
import frc.robot.commands.ShooterCommands.TeammatePassCommand;
import frc.robot.commands.ShooterCommands.TeammatePassFinishCommand;
import frc.robot.commands.ShooterCommands.UltrashotCommand;
import frc.robot.commands.ShooterCommands.ShooterIntakeCommands.IndexNoteInShooterCommand;
import frc.robot.commands.ShooterCommands.ShooterIntakeCommands.ShooterIntakeCommand;
import frc.robot.subsystems.AmpBarSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
    NamedCommands.registerCommand("Intake", intakeAuton().withTimeout(10));
    NamedCommands.registerCommand("ShootStation", getUltrashotAutonCommand());
    NamedCommands.registerCommand("ShooterSpin", new IdleShooterSpin(shooterSubsystem));
    NamedCommands.registerCommand("DisableRamp", new InstantCommand(() -> driveSubsystem.setDriveMotorRampRate(0)));
    NamedCommands.registerCommand("EnableRamp", new InstantCommand(() -> Commands.none()));
    NamedCommands.registerCommand("SetpointShot", getSetpointShotCommand());
  }

  public Command getUltrashotDrivingCommand() {
      return new UltrashotCommand(
      shooterSubsystem, driveSubsystem, ampBarSubsystem,
      () -> -driver.getLeftY() * 4.2,
      () -> -driver.getLeftX() * 4.2,
      operator::getLeftTriggerAxis
      );
  }

  public Command getUltrashotAutonCommand() {
    return new SequentialCommandGroup(
      new UltrashotAndSpinupCommand(shooterSubsystem, driveSubsystem).withTimeout(1.75),
      new UltrashotAndKickCommand(shooterSubsystem, driveSubsystem).withTimeout(0.5)
    );
  }
  
  public Command getInterpolationShootingCommand() {
    return new SequentialCommandGroup(
      new RotateTowardsTarget(shooterSubsystem, driveSubsystem, () -> driver.getLeftY() * 4.2, () -> driver.getLeftX() * 4.2),
      new InterpolationShootingCommand(shooterSubsystem, driveSubsystem, () -> driver.getLeftY() * 4.2, () -> driver.getLeftX() * 4.2, operator::getLeftTriggerAxis)
    );
  }

  public RobotContainer() {
    SmartDashboard.putNumber(Constants.P_thetaSmartdashboard, 0);
    SmartDashboard.putNumber(Constants.I_thetaSmartdashboard, 0);
    SmartDashboard.putNumber(Constants.D_thetaSmartdashboard, 0);

    registerCommands();
    SmartDashboard.putNumber("ThetaConstant", 0);
    PathLoader.configureAutoBuilder(driveSubsystem);

    driveSubsystem.setDriveMotorRampRate(0);
    driveSubsystem.setDefaultCommand(new FieldDrive(
    driveSubsystem,
    () -> driver.getLeftY() * 4.2,
    () -> driver.getLeftX() * 4.2,
    () -> driver.getRightX() * 2 * Math.PI));

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
    JoystickButton teammatePass = new JoystickButton(driver, 6);

    toggleIntake.toggleOnTrue(intake());
    
    zeroGyro.onTrue(new InstantCommand(driveSubsystem::resetGyroFieldDrive));

    ampMode.toggleOnTrue(getAmpScoringCommand(operator::getRightTriggerAxis, operator::getLeftTriggerAxis));
    ampMode.toggleOnFalse(new RetractAmpBarCommand(ampBarSubsystem));

    subwooferMode.toggleOnTrue(new SubwooferScoringCommand(shooterSubsystem, operator::getLeftTriggerAxis).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    retractIntake.whileTrue(new ForceIntakeUpCommand(intake));
    forceOutIntake.toggleOnTrue(new ReverseIntakeCommand(intake));

    teammatePass.toggleOnTrue(new TeammatePassCommand(shooterSubsystem));
    teammatePass.toggleOnFalse(new TeammatePassFinishCommand(shooterSubsystem).withTimeout(.25));

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
        new ShooterLineupCommand(intake, shooterSubsystem).withTimeout(1.0),
        new ShooterTransferCommand(intake, shooterSubsystem),
        new ShooterFinishCommand(shooterSubsystem).withTimeout(.2)
        // new SendbackCommand(shooterSubsystem).withTimeout(.1) 
      ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command intakeAuton() {
   return new SequentialCommandGroup(
        new IntakeStateCommand(intake, shooterSubsystem),
        new RunCommand(() -> intake.setIntakeSpin(1), intake).withTimeout(.05),
        new ShooterLineupCommand(intake, shooterSubsystem).withTimeout(0.75),
        new ShooterTransferCommand(intake, shooterSubsystem),
        new ShooterFinishCommand(shooterSubsystem).withTimeout(.2)
        // new SendbackCommand(shooterSubsystem).withTimeout(.1) 
      ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command getAmpScoringCommand(DoubleSupplier flywheel, DoubleSupplier kicker) {
    return new ParallelCommandGroup(new AmpScoringCommand(shooterSubsystem, flywheel, kicker), new ExtendAmpBarCommand(ampBarSubsystem)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command getInterpolationShotCommandAuton() {
    return new SequentialCommandGroup(
      new RotateTowardsTarget(shooterSubsystem, driveSubsystem, () -> 0, () -> 0).withTimeout(2),
      new InterpolateAndSpinupCommand(shooterSubsystem, driveSubsystem).withTimeout(1.0),
      new InterpolateAndKickCommand(shooterSubsystem, driveSubsystem).withTimeout(.25)
    );
  }

  public Command shooterIntakeCommand() {
    return new SequentialCommandGroup(
      new ShooterIntakeCommand(shooterSubsystem),
      new IndexNoteInShooterCommand(shooterSubsystem)
    );
  }
}
