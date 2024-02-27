// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.LimelightHelpers.LimelightResults;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class Limelight {
    private static final String backLimelight = "";
    private static final String frontLimelight = "limelight-cavbots";

    public static Pose2d getPose2d(String limelightName) {
        if(!targetBlue()) {
            return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        }
        return LimelightHelpers.getBotPose2d_wpiRed(limelightName);
    }

    public static Pose2d getPose2d() {
        return getPose2d(getMostAccurateLimelightName());
    }

    public static boolean targetBlue() {
        Alliance alliance = Alliance.Red;
        try {
            alliance = DriverStation.getAlliance().get();
        } catch(Exception e) {}

        return (alliance == Alliance.Blue); 
    }

    public static Pose2d[] getPoses() {
        Pose2d[] arr = new Pose2d[2];
        arr[0] = getPose2d(frontLimelight);
        arr[1] = getPose2d(backLimelight);
        return arr;
    }

    public static double[] getLatencies() {
       double[] arr = new double[2];
       arr[0] = LimelightHelpers.getLatency_Pipeline(frontLimelight);
       arr[1] = LimelightHelpers.getLatency_Pipeline(backLimelight);
       return arr;
    }

    public static int getTargetCount(String limelightName) {
        // return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "test").length;
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        return results.targetingResults.targets_Fiducials.length;
    }

    public static LimelightTarget_Fiducial getTargetFromID(String limelightName, int id) {
        LimelightTarget_Fiducial[] results = getVisibleTargets(limelightName);
        for(LimelightTarget_Fiducial fid: results) {
            if(fid.fiducialID == id) {
                return fid;
            }
        }
        return null;
    }

    public static double getTX(String limelightName) {
        return LimelightHelpers.getTX(limelightName);
    }

    public static double getTY(String limelightName) {
        return LimelightHelpers.getTY(limelightName);
    }

    public static double getDistanceToApriltag(LimelightTarget_Fiducial tag) {
        Pose3d pose = tag.getTargetPose_RobotSpace();
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getZ();

        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public static LimelightTarget_Fiducial[] getVisibleTargets(String limelightName) {
        return LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
    }

    public static double getAverageDistanceToAvailableTarget(String limelightName) {
        double sum = 0;
        LimelightTarget_Fiducial[] fiducials = getVisibleTargets(limelightName);

        for(LimelightTarget_Fiducial fid: fiducials) {
            sum += getDistanceToApriltag(fid);
        }

        return sum / fiducials.length;
    }

    public static double getAccuracyScore(String limelightName) {
        double score = getTargetCount(limelightName) / getAverageDistanceToAvailableTarget(limelightName);
        return (Double.isNaN(score)) ? 0: score;
    }

    public static double getOmegaRadToTag(int tagId, ChassisSpeeds lastSpeeds) {
        LimelightTarget_Fiducial target = getTargetFromID("", 4);
        if(target == null || lastSpeeds == null) {return 0.0;}
        Pose2d relative = target.getRobotPose_TargetSpace2D();

        double x = relative.getX();
        double y = relative.getY();

        double vx = lastSpeeds.vxMetersPerSecond;
        double vy = lastSpeeds.vyMetersPerSecond;

        SmartDashboard.putString("x, y", x + " ," + y);

        return (vx * y - vy * x) / (Math.pow(x, 2) + Math.pow(y, 2));
   
    }

    public static String getMostAccurateLimelightName() {
        double backAccuracy = getAccuracyScore(backLimelight);
        double frontAccuracy = getAccuracyScore(frontLimelight);

        return (backAccuracy > frontAccuracy) ? backLimelight: frontLimelight;
        // return backLimelight;
    }
}