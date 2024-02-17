// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.CycloidLibrary.NeoSteveModule;
import frc.robot.vision.Limelight;

public class DriveSubsystem extends SubsystemBase {
  NeoSteveModule fleft, fright, bleft, bright;

  Pigeon2 pigeon = new Pigeon2(Constants.PIGEON_ID, Constants.CANIVORE);
  PIDController headingController = new PIDController(4.26, .2, .1);
  

  SwerveDriveOdometry odometry;
  Field2d field;
  double currentOffset = 0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    fleft = new NeoSteveModule(Constants.FLEFT_DRIVE_ID, Constants.FLEFT_STEER_ID, Constants.FLEFT_CANCODER_ID, SwerveConstants.FLEFT_OFFSET, Constants.CANIVORE);
    fright = new NeoSteveModule(Constants.FRIGHT_DRIVE_ID, Constants.FRIGHT_STEER_ID, Constants.FRIGHT_CANCODER_ID, SwerveConstants.FRIGHT_OFFSET, Constants.CANIVORE);
    bleft = new NeoSteveModule(Constants.BLEFT_DRIVE_ID, Constants.BLEFT_STEER_ID, Constants.BLEFT_CANCODER_ID, SwerveConstants.BLEFT_OFFSET, Constants.CANIVORE);
    bright = new NeoSteveModule(Constants.BRIGHT_DRIVE_ID, Constants.BRIGHT_STEER_ID, Constants.BRIGHT_CANCODER_ID, SwerveConstants.BRIGHT_OFFSET, Constants.CANIVORE);

    odometry = new SwerveDriveOdometry(SwerveConstants.m_kinematics, getAngle(), getSwerveModulePositions());
    headingController.enableContinuousInput(0, 2*Math.PI);
    field = new Field2d();
    SmartDashboard.putData("fiel ldl dldd vd", field);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    fleft.setModuleState(states[0]);
    fright.setModuleState(states[1]);
    bleft.setModuleState(states[2]);
    bright.setModuleState(states[3]);
  }

  public void resetHeadingPID() {
    headingController.reset();
  }

  public double clamp(double x, double min, double max) {
    return (x > max) ? max: (x < min) ? min: x;
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = SwerveConstants.m_kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void autonDrive(ChassisSpeeds speeds) { //Man, pathplanner is weird
    ChassisSpeeds speeds2 = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
    drive(speeds2);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      fleft.getSwerveModulePosition(), 
      fright.getSwerveModulePosition(), 
      bleft.getSwerveModulePosition(), 
      bright.getSwerveModulePosition()};
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      fleft.getSwerveModuleState(),
      fright.getSwerveModuleState(),
      bleft.getSwerveModuleState(),
      fright.getSwerveModuleState()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] currentStates = getSwerveModuleStates();
    return SwerveConstants.m_kinematics.toChassisSpeeds(currentStates);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-pigeon.getAngle());
  }

  public Rotation2d getFieldDriveAngle() {
    return Rotation2d.fromDegrees(getAngle().getDegrees() - currentOffset);
  }

  public void resetGyroFieldDrive() {
    currentOffset = getAngle().getDegrees();
  }

  
  public void updateOdometry() {
      String accurate = Limelight.getMostAccurateLimelightName();
      SmartDashboard.putString("Most accurate lime", accurate);
      int targetAmount = Limelight.getTargetCount(accurate);
  
      SmartDashboard.putNumber("realGyroValue", getAngle().getDegrees());
      SmartDashboard.putNumber("LimelightAngle", Limelight.getPose2d(accurate).getRotation().getDegrees());
  
      if(targetAmount >= 2) {
        SmartDashboard.putBoolean("UsingLimelight", true);
        // System.out.println("UPDATING WITH LIMELIGHT");
        Pose2d limePose = Limelight.getPose2d(accurate);
        if(!(limePose.getX() == 0 && limePose.getY() == 0)) {
          updateOdometry(limePose);
          setYaw(limePose.getRotation().getDegrees());
          return;
        }
      }
  
      SmartDashboard.putBoolean("UsingLimelight", false);
      odometry.update(getAngle(), getSwerveModulePositions());
  }

  public void zeroGyro() {
    pigeon.reset();
  }
  public void updateOdometry(Pose2d pose) {
    odometry.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
  }

  public Pose2d getOdometry() {
    return odometry.getPoseMeters();
  }

  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
  }

  public void driveWithAngleOverride(Rotation2d angle, double xSpeed, double ySpeed) {
    Rotation2d currentAngle = getAngle();
    pushMeasurementAndSetpoint(angle.getRadians());
    double rotSpeeds = headingController.calculate(currentAngle.getRadians(), angle.getRadians());
    rotSpeeds = clamp(rotSpeeds, -1, 1);
    
    ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, -rotSpeeds), currentAngle);
    SmartDashboard.putNumber("DesiredXVelocity", fieldRelative.vxMetersPerSecond);
    SmartDashboard.putNumber("DesiredYVelocity", fieldRelative.vyMetersPerSecond);

    drive(fieldRelative);
  }

  public void pushMeasurementAndSetpoint(double setpoint) {
    SmartDashboard.putNumber("CurrentTheta", getAngle().getRadians());
    SmartDashboard.putNumber("SetpointTheta", setpoint);
    SmartDashboard.putNumber("ThetaError", getAngle().getRadians() - setpoint);
  }

  public void updateShooter() {
     headingController.setP(SmartDashboard.getNumber(Constants.P_thetaSmartdashboard, 0));
     headingController.setI(SmartDashboard.getNumber(Constants.I_thetaSmartdashboard, 0));
     headingController.setD(SmartDashboard.getNumber(Constants.D_thetaSmartdashboard, 0));

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FLEFT", fleft.getEncoderPosition());
    SmartDashboard.putNumber("FRIGHT", fright.getEncoderPosition());
    SmartDashboard.putNumber("BLEFT", bleft.getEncoderPosition());
    SmartDashboard.putNumber("BRIGHT", bright.getEncoderPosition());
    headingController.setP(SmartDashboard.getNumber("Bot_theta_P", 0));

    updateOdometry();
    field.setRobotPose(odometry.getPoseMeters());
  }
}
