// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class PoseTimestampPair {
    Pose2d pose;
    double latency;
    
    public PoseTimestampPair(Pose2d pose, double latency) {
        this.pose = pose;
        this.latency = latency;
    }

    public Pose2d getPose2d() {
        return this.pose;
    }

    public double getTimestamp() {
        return this.latency;
    }
}
