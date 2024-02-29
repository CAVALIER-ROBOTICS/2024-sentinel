// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  DriveSubsystem driveSubsystem;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    driveSubsystem = m_robotContainer.getDriveSubsystem();
    PathLoader.initSendableChooser();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    String pathName = PathLoader.getChosenAuton();
    m_autonomousCommand = PathLoader.loadAuto(pathName);
      Pose2d initial = PathPlannerAuto.getStaringPoseFromAutoFile(pathName);
      driveSubsystem.updateOdometry(initial);
      driveSubsystem.setYaw(initial.getRotation().getDegrees());
      driveSubsystem.updatePoseEstimator(initial);
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    driveSubsystem.setDriveMotorRampRate(Constants.SwerveConstants.DRIVE_MOTOR_RAMP_RATE);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
