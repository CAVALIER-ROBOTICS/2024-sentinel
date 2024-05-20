// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interpolation;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import frc.robot.Interpolation.ShotParam;
import static java.util.Map.entry;
/** Add your docs here. */
public class InterpolatingTable {
    private static final TreeMap<Double, ShotParam> map = new TreeMap<>(Map.ofEntries(
        entry(0.0, new ShotParam(0, 0)),
        entry(1.0, new ShotParam(1, 1)),
        entry(2.0, new ShotParam(0, 0)))
    );

    public static ShotParam getShotParameter(Double distance) {
        Entry<Double, ShotParam> floor = map.floorEntry(distance);
        Entry<Double, ShotParam> ceil = map.ceilingEntry(distance);
        ShotParam ceilValue = ceil.getValue();
        ShotParam floorValue = floor.getValue();
        if(ceilValue == null) {
            return floorValue;
        }
        if(floorValue == null) {
            return ceilValue;
        }
        if(ceilValue.equals(floorValue)) {
            return ceilValue;
        }

        double percentAlong = (distance - floor.getKey()) / (ceil.getKey() - floor.getKey());
        return floorValue.interpolate(percentAlong, ceilValue);
    }
}
