// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.vision.LimelightHelpers.LimelightResults;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class Limelight {
    public static final String limelightname = "limelight-cavbots";

    public static Pose2d getPose2d(String limelightName) {
        // if(!targetBlue()) {
        //     return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        // }

        return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }

    public static boolean targetBlue() {
        Alliance alliance = Alliance.Red;
        Optional<Alliance> allianceLol = DriverStation.getAlliance();
        
        if(allianceLol.isEmpty()) {
            return false;
        }
        alliance = allianceLol.get();
        return (alliance == Alliance.Blue); 
    }

    public static double getCombinedLantencySeconds(String name) {
        return (LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name)) / 1000;
    }

    public static boolean canLimelightProvideAccuratePoseEstimate(String name) {
        int targetcount = getTargetCount(name);
        double avgdist = getAverageDistanceToAvailableTarget(name);
        return ((targetcount >= 2 || avgdist < Constants.MAX_DISTANCE_TO_SINGLETAG) && avgdist < Constants.MAX_DISTANCE_TO_APRILTAG) && targetcount > 0;
        // return true;
    }

    public static boolean canLocalizeWithEstimatorReset(String name) {
        int targetcount = getTargetCount(name);
        double avgdist = getAverageDistanceToAvailableTarget(name);
        return targetcount >= 2 && avgdist < Constants.MAX_DISTANCE_TO_APRILTAG;
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

    public static int getCentralTagId() {
        if(Limelight.targetBlue()) {
            return 7;
        }
        return 4;
    }

    public static double getDistanceToTargetTag() {
        int centralID = getCentralTagId();
        LimelightTarget_Fiducial fid = getTargetFromID(limelightname, centralID);

        if(fid != null) {
            double tagHeight = fid.ty;
            double bruh = (1.4478 - .6858) / Math.tan(Math.toRadians(20 + tagHeight));
            return bruh;
        }

        return -1.0;
    }

    public static double getTargetTagCenterOffsetX() {
        int centralID = getCentralTagId();
        LimelightTarget_Fiducial fid = getTargetFromID(limelightname, centralID);

        if(fid != null) {
           return fid.tx;
        }
        return 0.0;
    }

    public static VisionTarget getTagVisionTargetPercent(String name, int id) {
        LimelightTarget_Fiducial fid = getTargetFromID(name, id);
        if(fid != null) {
            return new VisionTarget(fid.tx_pixels / 960, fid.ty_pixels / 720);
        }
        return new VisionTarget(0, 0);
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
}