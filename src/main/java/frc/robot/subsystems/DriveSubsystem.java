// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PiConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.CycloidLibrary.NeoSteveModule;
import frc.robot.ultrashot.Point2D;
import frc.robot.ultrashot.pose.poseestimator;
import frc.robot.vectorfields.Riptide;
import frc.robot.vision.Limelight;
import frc.robot.vision.VisionTarget;

public class DriveSubsystem extends SubsystemBase {
  NeoSteveModule fleft, fright, bleft, bright;

  Pigeon2 pigeon = new Pigeon2(Constants.PIGEON_ID, Constants.CANIVORE);
  double headingP = 3.0;
  PIDController headingController = new PIDController(headingP, 0.01, .15);
  PIDController limelightHeadingController = new PIDController(1, 0, 0);
  
  
  SwerveDrivePoseEstimator estimator;

  Field2d haydenField;
  Field2d poseEstimatorField;

  double currentOffset = 0;

  poseestimator haydenEstimator;
  Riptide riptide;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    fleft = new NeoSteveModule(Constants.FLEFT_DRIVE_ID, Constants.FLEFT_STEER_ID, Constants.FLEFT_CANCODER_ID, SwerveConstants.FLEFT_OFFSET, Constants.CANIVORE);
    fright = new NeoSteveModule(Constants.FRIGHT_DRIVE_ID, Constants.FRIGHT_STEER_ID, Constants.FRIGHT_CANCODER_ID, SwerveConstants.FRIGHT_OFFSET, Constants.CANIVORE);
    bleft = new NeoSteveModule(Constants.BLEFT_DRIVE_ID, Constants.BLEFT_STEER_ID, Constants.BLEFT_CANCODER_ID, SwerveConstants.BLEFT_OFFSET, Constants.CANIVORE);
    bright = new NeoSteveModule(Constants.BRIGHT_DRIVE_ID, Constants.BRIGHT_STEER_ID, Constants.BRIGHT_CANCODER_ID, SwerveConstants.BRIGHT_OFFSET, Constants.CANIVORE);

    bright.setSteerP(.15);
    estimator = new SwerveDrivePoseEstimator(SwerveConstants.m_kinematics, getAngle(), getSwerveModulePositions(), new Pose2d());

    headingController.enableContinuousInput(0, 2 * Math.PI);
    poseEstimatorField = new Field2d();
    haydenField = new Field2d();

    haydenEstimator = new poseestimator(62.5, 48.9);

    riptide = new Riptide();

    SmartDashboard.putData("HaydenField", haydenField);
    SmartDashboard.putData("fiedd", poseEstimatorField);
  }

  public void updateRiptide() {
    Pose2d pose = getEstimatedPosition();
    Point2D point = new Point2D(pose.getX(), pose.getY());

    riptide.update(point, new Point2D[0], new Point2D[0]);
  }

  public Point2D getRiptideSpeeds() {
    return riptide.getVelocity();
  }

  public Pose2d getHaydenEstimatorPose2d() {
    haydenEstimator.setAlliance(!Limelight.targetBlue());
    VisionTarget target = Limelight.getTagVisionTargetPercent(Limelight.limelightname, Limelight.getCentralTagId());
    haydenEstimator.update(target.getX(), target.getY(), getAngle().getRadians());
    return new Pose2d(haydenEstimator.getRobotX(), haydenEstimator.getRobotY(), getAngle());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    fleft.setModuleState(states[0]);
    fright.setModuleState(states[1]);
    bleft.setModuleState(states[2]);
    bright.setModuleState(states[3]);
    // fleft.setPercentOutput(.5);
    // fright.setPercentOutput(.5);
    // bleft.setPercentOutput(.5);
    // bright.setPercentOutput(.5);
  }

  public void setDriveMotorRampRate(double ramp) {
    fleft.configureRampRate(ramp);
    fright.configureRampRate(ramp);
    bleft.configureRampRate(ramp);
    bright.configureRampRate(ramp);
  }

  public void resetHeadingPID() {
    headingController.reset();
  }

  private double clamp(double x, double min, double max) {
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

  public double getRobotVelocityMagnitude() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
  }

  public double getPScalingFactor() {
    double percent = getRobotVelocityMagnitude() / 4.2;
    return percent;
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
 
  public void updatePoseEstimator() {
    String llname = Limelight.limelightname;
    Pose2d pose = Limelight.getPose2d(llname);
    boolean canAddMeasurement = Limelight.canLocalizeWithEstimatorReset(llname);
    SmartDashboard.putBoolean("Adding measurements", canAddMeasurement);
    SmartDashboard.putNumber("TargAmount", Limelight.getTargetCount(llname));
    
    // estimator.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
    if(canAddMeasurement && pose.getX() != 0 && pose.getY() != 0) {
      // estimator.addVisionMeasurement(pose, Limelight.getCombinedLantencySeconds(llname), VecBuilder.fill(0, 0, 0));
      estimator.resetPosition(getAngle(), getSwerveModulePositions(), pose);
      setYaw(pose.getRotation().getDegrees());
    }
    
    estimator.update(getAngle(), getSwerveModulePositions());
  }

  public void zeroGyro() {
    pigeon.reset();
  }

  public void updatePoseEstimator(Pose2d pose) {
    estimator.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
  }

  public Pose2d getEstimatedPosition() {
    return estimator.getEstimatedPosition();
  }

  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
  }

  public void updatePWithBotVelocity() {
    headingController.setP(getPScalingFactor() + headingP);
  }

  public void driveWithAngleOverride(Rotation2d angle, double xSpeed, double ySpeed, double omega) {
    Rotation2d currentAngle = getAngle();
    double rotSpeeds = (headingController.calculate(currentAngle.getRadians(), angle.getRadians()) + SmartDashboard.getNumber("ThetaConstant", 0) * omega);
    rotSpeeds = clamp(rotSpeeds, -3, 3);

    ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, -rotSpeeds * (getPScalingFactor() + 1)), getFieldDriveAngle());
    drive(fieldRelative);
    pushMeasurementAndSetpoint(angle.getRadians());
  }

  public void driveWithApriltagCentering(double x, double y) {
    double measurement = Limelight.getTargetTagCenterOffsetX();
    double setpoint = limelightHeadingController.calculate(measurement, 0.0);

    drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, setpoint), getAngle()));
  }

  public void pushMeasurementAndSetpoint(double setpoint) {
    SmartDashboard.putNumber("CurrentTheta", getAngle().getRadians());
    SmartDashboard.putNumber("SetpointTheta", setpoint);
    SmartDashboard.putNumber("ThetaError", (getAngle().getRadians() - setpoint) % Math.PI * 2);
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
    SmartDashboard.putNumber("Gyro angle rads", getAngle().getRadians());
    // updateRiptide();

    updatePoseEstimator();
    // updateShooter();

    poseEstimatorField.setRobotPose(estimator.getEstimatedPosition());
    haydenField.setRobotPose(getHaydenEstimatorPose2d());
  }
}
