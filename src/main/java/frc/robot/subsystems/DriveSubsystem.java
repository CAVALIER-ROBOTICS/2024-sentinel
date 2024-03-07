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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.CycloidLibrary.NeoSteveModule;
import frc.robot.ultrashot.Point2D;
import frc.robot.ultrashot.pose.poseestimator;
import frc.robot.vectorfields.VectorFieldGenerator;
import frc.robot.vision.JetsonHandler;
import frc.robot.vision.Limelight;
import frc.robot.vision.PiHandler;
import frc.robot.vision.VisionTarget;
import frc.robot.vectorfields.*;

public class DriveSubsystem extends SubsystemBase {
  NeoSteveModule fleft, fright, bleft, bright;

  Pigeon2 pigeon = new Pigeon2(Constants.PIGEON_ID, Constants.CANIVORE);
  PIDController headingController = new PIDController(4.26, 0.0, 0.1);
  

  SwerveDriveOdometry odometry;
  SwerveDrivePoseEstimator estimator;

  Field2d odometryField;
  Field2d haydenField;
  Field2d poseEstimatorField;

  VectorFieldGenerator vectorField;

  double currentOffset = 0;

  poseestimator haydenEstimator;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    fleft = new NeoSteveModule(Constants.FLEFT_DRIVE_ID, Constants.FLEFT_STEER_ID, Constants.FLEFT_CANCODER_ID, SwerveConstants.FLEFT_OFFSET, Constants.CANIVORE);
    fright = new NeoSteveModule(Constants.FRIGHT_DRIVE_ID, Constants.FRIGHT_STEER_ID, Constants.FRIGHT_CANCODER_ID, SwerveConstants.FRIGHT_OFFSET, Constants.CANIVORE);
    bleft = new NeoSteveModule(Constants.BLEFT_DRIVE_ID, Constants.BLEFT_STEER_ID, Constants.BLEFT_CANCODER_ID, SwerveConstants.BLEFT_OFFSET, Constants.CANIVORE);
    bright = new NeoSteveModule(Constants.BRIGHT_DRIVE_ID, Constants.BRIGHT_STEER_ID, Constants.BRIGHT_CANCODER_ID, SwerveConstants.BRIGHT_OFFSET, Constants.CANIVORE);

    bright.setSteerP(.1);
    odometry = new SwerveDriveOdometry(SwerveConstants.m_kinematics, getAngle(), getSwerveModulePositions());
    estimator = new SwerveDrivePoseEstimator(SwerveConstants.m_kinematics, getAngle(), getSwerveModulePositions(), new Pose2d());

    headingController.enableContinuousInput(0, 2 * Math.PI);
    odometryField = new Field2d();
    poseEstimatorField = new Field2d();
    haydenField = new Field2d();


    haydenEstimator = new poseestimator(62.5, 48.9);

    vectorField = new VectorFieldGenerator();
    vectorField.configure(
      VectorFieldConstants.attraction,
      VectorFieldConstants.repulsion,
      VectorFieldConstants.node,
      VectorFieldConstants.antiNodes
    );

    SmartDashboard.putData("OdometryField", odometryField);
    SmartDashboard.putData("HaydenField", haydenField);
    SmartDashboard.putData("PoseEstimatorField", poseEstimatorField);
  }

  public Pose2d getHaydenEstimatorPose2d() {
    haydenEstimator.setAlliance(!Limelight.targetBlue());
    VisionTarget target = Limelight.getTagVisionTargetPercent(Limelight.getCentralTagId());
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

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-pigeon.getAngle());
  }

  public Rotation2d getFieldDriveAngle() {
    return Rotation2d.fromDegrees(getAngle().getDegrees() - currentOffset);
  }

  public VectorFieldGenerator getVectorFieldGenerator() {
    return this.vectorField;
  }

  public void resetGyroFieldDrive() {
    currentOffset = getAngle().getDegrees();
  }

  
  public void updateOdometry() {
      String accurate = Limelight.getMostAccurateLimelightName();
      int targetAmount = Limelight.getTargetCount("");
      SmartDashboard.putBoolean("autonomous", DriverStation.isAutonomous());
      SmartDashboard.putNumber("targetAmount", targetAmount);
      SmartDashboard.putNumber("Gyro angle", getAngle().getDegrees());
      SmartDashboard.putNumber("averagedist", Limelight.getAverageDistanceToAvailableTarget(accurate));
      // if(targetAmount >= 2 && Limelight.getAverageDistanceToAvailableTarget(accurate) <= Constants.MAX_DISTANCE_TO_APRILTAG) {
      //   SmartDashboard.putBoolean("UsingLimelight", true);
      //   Pose2d limePose = Limelight.getPose2d(accurate);
      //   if(!(limePose.getX() == 0 && limePose.getY() == 0)) {
      //     limePose = new Pose2d(limePose.getX(), limePose.getY(), Rotation2d.fromDegrees(limePose.getRotation().getDegrees() + 180));
      //     updateOdometry(new Pose2d(limePose.getX(), limePose.getY(), getAngle()));
      //     // setYaw(limePose.getRotation().getDegrees());
      //     SmartDashboard.putNumber("lime rotation", limePose.getRotation().getDegrees());
      //     return;
      //   }
      // }
      Pose2d cool = JetsonHandler.getBotSlamPose();

      SmartDashboard.putBoolean("UsingLimelight", false);
      odometry.resetPosition(getAngle(), getSwerveModulePositions(), cool);
  }
  //test lol
  public void updatePoseEstimator() {
    Pose2d[] estimates = Limelight.getPoses();
    double[] latencies = Limelight.getLatencies();

    estimator.update(getAngle(), getSwerveModulePositions());

    for(int i = 0; i < estimates.length; i++) {
      Pose2d pose = estimates[i];
      double latencySeconds = latencies[i] / 1000;

      if(pose.getX() == 0 && pose.getY() == 0) {
        continue;
      }

      estimator.addVisionMeasurement(pose, latencySeconds);
    }
  }

  public void zeroGyro() {
    pigeon.reset();
  }

  public void updateOdometry(Pose2d pose) {
    odometry.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
  }

  public void updatePoseEstimator(Pose2d pose) {
    estimator.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
  }

  public Pose2d getOdometry() {
    return odometry.getPoseMeters();
  }

  public Pose2d getEstimatedPosition() {
    return estimator.getEstimatedPosition();
  }

  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
  }

  public void driveWithAngleOverride(Rotation2d angle, double xSpeed, double ySpeed, double omega) {
    Rotation2d currentAngle = getAngle();
    pushMeasurementAndSetpoint(angle.getRadians());
    double rotSpeeds = headingController.calculate(currentAngle.getRadians(), angle.getRadians()) + headingController.getD() * omega;
    rotSpeeds = clamp(rotSpeeds, -3, 3);
    SmartDashboard.putNumber("OmegaNutsLol", omega);
    ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, -rotSpeeds), currentAngle);

    drive(fieldRelative);
    pushMeasurementAndSetpoint(angle.getRadians());
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

    updateOdometry();
    updatePoseEstimator();
    vectorField.update(new Point2D(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY()));


    odometryField.setRobotPose(odometry.getPoseMeters());
    poseEstimatorField.setRobotPose(estimator.getEstimatedPosition());
    haydenField.setRobotPose(getHaydenEstimatorPose2d());
  }
}
