// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimelightCustom {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("");

    public static double getTX() {
        return table.getEntry("tx").getDouble(-1);
    }
}
