// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import java.util.Map;
import java.util.TreeMap;

/** Add your docs here. */
public class LookupTable {

    static final TreeMap<Double, Double> table = new TreeMap<>();

    public LookupTable(double[] data) {

        for (int i=0; i < data.length; i += 2) {

            table.put(data[i], data[i + 1]);
        }

    }

    public static double get(double distance) {

        Map.Entry<Double, Double> lower = table.floorEntry(distance);
        Map.Entry<Double, Double> upper = table.ceilingEntry(distance);

        if (lower == null) { lower = table.firstEntry(); }
        if (upper == null) { upper = table.lastEntry(); }

        Double d1 = lower.getKey();
        Double v1 = lower.getValue();
        double d2 = upper.getKey();
        double v2 = upper.getValue();

        return v1 + (distance - d1)*(v2 - v1)/(d2-d1);

    }

}
