package org.firstinspires.ftc.teamcode.util;

import java.util.TreeMap;

public class InterpolatingTable {
    private final TreeMap<Double, Double> map = new TreeMap<>();

    public void put(double key, double value) {
        map.put(key, value);
    }

    public double get(double key) {
        if (map.isEmpty()) return 0.0;

        Double lowerKey = map.floorKey(key);
        Double upperKey = map.ceilingKey(key);

        // Clamp to nearest endpoint if outside measured range
        if (lowerKey == null) return map.get(upperKey);
        if (upperKey == null) return map.get(lowerKey);
        if (lowerKey.equals(upperKey)) return map.get(lowerKey);

        double lowerVal = map.get(lowerKey);
        double upperVal = map.get(upperKey);
        double t = (key - lowerKey) / (upperKey - lowerKey);

        return lowerVal + t * (upperVal - lowerVal);
    }
}