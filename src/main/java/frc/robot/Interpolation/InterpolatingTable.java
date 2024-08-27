// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import static java.util.Map.entry;
/** Add your docs here. */
public class InterpolatingTable {
    private static final TreeMap<Double, ShotParam> map = new TreeMap<>(Map.ofEntries(
        entry(1.4, new ShotParam(52, .5)),
        entry(2.0, new ShotParam(46.5, .5)), //51
        entry(2.2, new ShotParam(44.5, .5)),
        entry(2.3, new ShotParam(42.5, .5)),
        entry(2.4, new ShotParam(42.5, .5)),
        entry(2.6, new ShotParam(42.675, .5)),
        entry(3.0, new ShotParam(38.5, .5)),
        entry(3.2, new ShotParam(37.5, .5)),
        entry(3.4, new ShotParam(37, .5)),
        entry(3.64, new ShotParam(35, 1)), //check this
        entry(4.0, new ShotParam(34, 1)),
        entry(4.6, new ShotParam(32, 1))

      )
    );

    public static ShotParam getShotParameter(Double distance) {
        Entry<Double, ShotParam> floor = map.floorEntry(distance);
        Entry<Double, ShotParam> ceil = map.ceilingEntry(distance);
        
        if(ceil == null) {
            return floor.getValue();
        }
        if(floor == null) {
            return ceil.getValue();
        }
        if(ceil.getValue().equals(floor.getValue())) {
            return ceil.getValue();
        }

        double percentAlong = (distance - floor.getKey()) / (ceil.getKey() - floor.getKey());
        return floor.getValue().interpolate(percentAlong, ceil.getValue());
    }
}
