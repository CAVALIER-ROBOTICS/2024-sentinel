// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class PathLoader {
    static PIDController x = new PIDController(0.0, 0.0, 0.0);
    static PIDController y = new PIDController(0.0, 0.0, 0.0);
    static PIDController theta = new PIDController(0.0, 0.0, 0.0);

    static SendableChooser<String> chooser = new SendableChooser<String>();

    static String[] validAutonPaths = {
        "redside",
        "blueside",
        "shootwhatugotred",
        "shootwhatugotblue",
        "BLUEBOTTOM_2note",
        "BLUETOP_2note",
        "BLUEMID_2note",
        "REDMID_2note",
        "REDBOTTOM_2note",
        "REDTOP_2note",
        "simple3note",
        "BLUE_4NOTE",
        "RED_4NOTE"
    };

    public static PathPlannerPath getPath(String path) {
        return PathPlannerPath.fromPathFile(path);
    }

    public static Boolean getShouldFlipPath() {
        return false;
    }

    public static void configureAutoBuilder(DriveSubsystem driveSub) {
        Consumer<Pose2d> resetPose = pose -> {
            driveSub.updatePoseEstimator(pose);
        };

        Consumer<ChassisSpeeds> drivelol = speeds -> driveSub.autonDrive(speeds);

        HolonomicPathFollowerConfig hpfc = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0),
            new PIDConstants(4.0, 0.0, 0.1),
            4.2,
            SwerveConstants.BOT_LENGTH / 2,
            new ReplanningConfig(),
            0.02
        );
        
        AutoBuilder.configureHolonomic(
                driveSub::getEstimatedPosition, 
                resetPose,
                driveSub::getChassisSpeeds,
                drivelol,
                hpfc,
                () -> getShouldFlipPath(),
                driveSub);
    }

    public static Command pathfindToPosition(Pose2d position) { //Make sure navgrid.json is in /deploy/pathplanner
        PathConstraints constraints = new PathConstraints(1.5, 3, Math.toRadians(540), Math.toRadians(720));
        Command path = AutoBuilder.pathfindToPose(position, constraints, 0, 0);
        return path;
    }

    public static void configureDynamicObstacles(Pose2d[] botObstacleCenters, Translation2d currentBotPos) {}

    public static Command loadAuto(String name) {
        return new PathPlannerAuto(name);
    }

    public static Command loadPath(String name) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(name);
        return AutoBuilder.followPath(path);
    }

    public static void initSendableChooser() {
        chooser.setDefaultOption(validAutonPaths[0], validAutonPaths[0]);
        for(String v: validAutonPaths) {
            if(v == validAutonPaths[0]) {
                continue;
            }
            chooser.addOption(v, v);
        }
        SmartDashboard.putData("Autonomous type", chooser);
    }

    public static String getAutoName() {
        return chooser.getSelected();
    }

    public static Command getChosenAuton() {
        String selected = getAutoName();
        SmartDashboard.putString("Selected auto", selected);
        return loadAuto(selected);
    }
}
