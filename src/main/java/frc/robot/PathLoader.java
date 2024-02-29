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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        "fournote_local",
        "fournote_local_blue"
    };

    public static PathPlannerPath getPath(String path) {
        return PathPlannerPath.fromPathFile(path);
    }

    public static Boolean getShouldFlipPath() {
        return (DriverStation.getAlliance().get() == Alliance.Blue);
    }

    public static void configureAutoBuilder(DriveSubsystem driveSub) {
        Consumer<Pose2d> resetPose = pose -> {
            driveSub.updatePoseEstimator(pose);
            driveSub.updateOdometry(pose);
        };

        Consumer<ChassisSpeeds> drivelol = speeds -> driveSub.autonDrive(speeds);

        HolonomicPathFollowerConfig hpfc = new HolonomicPathFollowerConfig(
            new PIDConstants(.2),
            new PIDConstants(4.26, 0.0, 0.1),
            4.2,
            SwerveConstants.BOT_LENGTH / 2,
            new ReplanningConfig(),
            0.02
        );
        
        AutoBuilder.configureHolonomic(
                driveSub::getEstimatedPosition, //TODO this uses the pose estimator now, idk how well it'll work
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
        chooser.setDefaultOption("Default", validAutonPaths[0]);
        for(String v: validAutonPaths) {
            if(v == validAutonPaths[0]) {
                continue;
            }
            chooser.addOption(v, v);
        }
        SmartDashboard.putData("Autonomous type", chooser);
    }

    public static String getChosenAuton() {
        return new String(chooser.getSelected());
    }
}
