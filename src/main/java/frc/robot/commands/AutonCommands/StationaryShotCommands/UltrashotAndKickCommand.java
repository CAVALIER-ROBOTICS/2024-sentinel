// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands.StationaryShotCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Interpolation.InterpolatingTable;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ultrashot.AngleStates;
import frc.robot.ultrashot.Point3D;
import frc.robot.ultrashot.UltraShotConstants;

public class UltrashotAndKickCommand extends Command {
  /** Creates a new UltrashotCommand. */
  ShooterSubsystem shooterSubsystem;
  DriveSubsystem driveSubsystem;

  public UltrashotAndKickCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.driveSubsystem = driveSubsystem;

    addRequirements(shooterSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetHeadingPID();
  }

   public Translation2d getGoalTranslation2d() {
    Point3D goal;
    if(DriverStation.getAlliance().get() == Alliance.Red) {
      goal = UltraShotConstants.POINT_3D_SPEAKER_RED;
    } else {
      goal = UltraShotConstants.POINT_3D_SPEAKER_BLUE;
    }
    return new Translation2d(goal.getX(), goal.getY());
  }

  public double getDistanceToTarget() {
    Translation2d botPose = driveSubsystem.getEstimatedPosition().getTranslation();
    Translation2d goalPose = getGoalTranslation2d();
    
    Translation2d diff = goalPose.minus(botPose);
    return Math.sqrt(Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2));
  }
  // Called every time the scheduler runs while the command is scheduled.


  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    shooterSubsystem.updateUltrashot(driveSubsystem);

    AngleStates states = shooterSubsystem.getAngleStates();

    if(Double.isNaN(states.getTheta())) {return;}

    driveSubsystem.driveWithAngleOverride(Rotation2d.fromRadians(states.getTheta() + Math.PI), 0, 0, 0); // 0.1 is the heading controller D
    shooterSubsystem.setKickerSpeed(-1);
    double dist = getDistanceToTarget();
    System.out.println(dist);
    shooterSubsystem.shootFromShotParameter(InterpolatingTable.getShotParameter(dist));

    // ampBarSubsystem.setPosition(Constants.AmpBarConstants.AMPBAR_EXTENDED / 2);
    SmartDashboard.putNumber("theta", states.getTheta());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return shooterSubsystem.getAverageRPM() >= Constants.ShooterConstants.MAX_RPM_FLYWHEEL;
    return false;
  }
}
