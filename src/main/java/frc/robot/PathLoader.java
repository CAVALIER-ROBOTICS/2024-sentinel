// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathLoader {
    static PIDController x = new PIDController(0.0, 0.0, 0.0);
    static PIDController y = new PIDController(0.0, 0.0, 0.0);
    static PIDController theta = new PIDController(0.0, 0.0, 0.0);

    public static PathPlannerPath getPath(String path) {
        return PathPlannerPath.fromPathFile(path);
    }

    public static Boolean getShouldFlipPath() {
        return false;
    }

    public static void configureAutoBuilder(DriveSubsystem driveSub) {
        Consumer<Pose2d> resetPose = pose -> driveSub.updateOdometry(pose);
        Consumer<ChassisSpeeds> drivelol = speeds -> driveSub.autonDrive(speeds);

        HolonomicPathFollowerConfig hpfc = new HolonomicPathFollowerConfig(
            new PIDConstants(.05, 0, 0),
            new PIDConstants(.05, 0, 0),
            4.2,
            Constants.BOT_LENGTH / 2,
            new ReplanningConfig(),
            0.02
        );
        
        AutoBuilder.configureHolonomic(
                driveSub::getOdometry,
                resetPose,
                driveSub::getChassisSpeeds,
                drivelol,
                hpfc,
                () -> getShouldFlipPath(),
                driveSub);
    }

    public static Command loadPath(String name) {
        return new PathPlannerAuto(name);
        // PathPlannerPath path = PathPlannerPath.fromPathFile(name);
        // return AutoBuilder.followPath(path);
    }
}
