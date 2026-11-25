package org.firstinspires.ftc.teamcode.util.paths;

import java.util.Arrays;

public class AutonomousPaths {
    public static double robotWidth = 12.4409;
    public static double robotHeight = 17.2;
    public static double[] initial = new double[]{48 + (robotWidth / 2), robotHeight / 2, 90}; // x, y, heading
    public static double[] down = new double[]{58, 33, 90};
    public static double[] up = new double[]{58, 111, 270};
    public static double[] flowerUp1 = new double[]{48, 115, 90};
    public static double[] flowerUp2 = new double[]{48, 133, 90};
    public static double[] flowerDown1 = new double[]{26, 48, 180};
    public static double[] flowerDown2 = new double[]{12.5, 48, 180};
    public static double[] garden1 = new double[]{23, 9, 200};
    public static double[] garden2 = new double[]{10, 9, 180};
    public static double[] park = new double[]{9, 95, 90};
    public static double[] upDownControl = new double[]{80, 72, 0};
    public static double[] upGardenControl = new double[]{6, 15, 0};
    public static double[] downFlowerUpControl = new double[]{0, 41, 0};
    public static double[] downParkControl = new double[]{29, 21, 0};
    public static double[] upFlowerDownControl = new double[]{12, 112, 0};

    public static String getPose(double[] pose) {
        if (Arrays.equals(pose, initial)) return "initial";
        if (Arrays.equals(pose, down)) return "down";
        if (Arrays.equals(pose, up)) return "up";
        if (Arrays.equals(pose, flowerUp1)) return "flowerUp1";
        if (Arrays.equals(pose, flowerUp2)) return "flowerUp2";
        if (Arrays.equals(pose, flowerDown1)) return "flowerDown1";
        if (Arrays.equals(pose, flowerDown2)) return "flowerDown2";
        if (Arrays.equals(pose, garden1)) return "garden1";
        if (Arrays.equals(pose, garden2)) return "garden2";
        if (Arrays.equals(pose, park)) return "park";

        return "unknown";
    }

    // pass through zz on:
    /*
     * up to down
     * down to flowerUp
     * up to flowerDown
     * up to garden
     */
}
