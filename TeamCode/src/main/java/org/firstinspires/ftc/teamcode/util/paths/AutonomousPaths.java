package org.firstinspires.ftc.teamcode.util.paths;

import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;

public class AutonomousPaths {
    public static final double FIELD_SIZE = 144.0;
    public static final double FIELD_MID_X = 72.0;
    public static final double FIELD_MID_Y = 72.0;

    public static double robotWidth = 12.4409;
    public static double robotHeight = 17.2;

    private static final double[] RED_INITIAL = new double[]{48 + (robotWidth / 2), robotHeight / 2, 90}; // x, y, heading
    private static final double[] RED_DOWN = new double[]{58, 33, 90};
    private static final double[] RED_UP = new double[]{58, 111, 270};
    private static final double[] RED_FLOWER_UP_1 = new double[]{48, 115, 90};
    private static final double[] RED_FLOWER_UP_2 = new double[]{48, 133, 90};
    private static final double[] RED_FLOWER_DOWN_1 = new double[]{26, 48, 180};
    private static final double[] RED_FLOWER_DOWN_2 = new double[]{12.5, 48, 180};
    private static final double[] RED_GARDEN_1 = new double[]{23, 9, 200};
    private static final double[] RED_GARDEN_2 = new double[]{10, 9, 180};
    private static final double[] RED_PARK = new double[]{9, 95, 90};
    private static final double[] RED_UP_DOWN_CONTROL = new double[]{80, 72, 0};
    private static final double[] RED_UP_GARDEN_CONTROL = new double[]{6, 15, 0};
    private static final double[] RED_DOWN_FLOWER_UP_CONTROL = new double[]{0, 41, 0};
    private static final double[] RED_DOWN_PARK_CONTROL = new double[]{29, 21, 0};
    private static final double[] RED_UP_FLOWER_DOWN_CONTROL = new double[]{12, 112, 0};
    private static final double[] RED_SEARCH_1 = new double[]{32, 36, 45};
    private static final double[] RED_SEARCH_2 = new double[]{28, 72, 0};
    private static final double[] RED_SEARCH_3 = new double[]{34, 108, 315};

    public static double[] initial = forTeam(RED_INITIAL);
    public static double[] down = forTeam(RED_DOWN);
    public static double[] up = forTeam(RED_UP);
    public static double[] flowerUp1 = forTeam(RED_FLOWER_UP_1);
    public static double[] flowerUp2 = forTeam(RED_FLOWER_UP_2);
    public static double[] flowerDown1 = forTeam(RED_FLOWER_DOWN_1);
    public static double[] flowerDown2 = forTeam(RED_FLOWER_DOWN_2);
    public static double[] garden1 = forTeam(RED_GARDEN_1);
    public static double[] garden2 = forTeam(RED_GARDEN_2);
    public static double[] park = forTeam(RED_PARK);
    public static double[] upDownControl = forTeam(RED_UP_DOWN_CONTROL);
    public static double[] upGardenControl = forTeam(RED_UP_GARDEN_CONTROL);
    public static double[] downFlowerUpControl = forTeam(RED_DOWN_FLOWER_UP_CONTROL);
    public static double[] downParkControl = forTeam(RED_DOWN_PARK_CONTROL);
    public static double[] upFlowerDownControl = forTeam(RED_UP_FLOWER_DOWN_CONTROL);
    public static double[] search1 = forTeam(RED_SEARCH_1);
    public static double[] search2 = forTeam(RED_SEARCH_2);
    public static double[] search3 = forTeam(RED_SEARCH_3);
    public static double[][] searchPoses = new double[][]{search1, search2, search3};

    public static double[] forTeam(double[] redPose) {
        if ("B".equals(PoseStorage.TEAM)) {
            double mirroredHeading = ((180.0 - redPose[2]) % 360.0 + 360.0) % 360.0;
            return new double[]{FIELD_SIZE - redPose[0], redPose[1], mirroredHeading};
        }
        return new double[]{redPose[0], redPose[1], redPose[2]};
    }

    public static void updateForTeam() {
        initial = forTeam(RED_INITIAL);
        down = forTeam(RED_DOWN);
        up = forTeam(RED_UP);
        flowerUp1 = forTeam(RED_FLOWER_UP_1);
        flowerUp2 = forTeam(RED_FLOWER_UP_2);
        flowerDown1 = forTeam(RED_FLOWER_DOWN_1);
        flowerDown2 = forTeam(RED_FLOWER_DOWN_2);
        garden1 = forTeam(RED_GARDEN_1);
        garden2 = forTeam(RED_GARDEN_2);
        park = forTeam(RED_PARK);
        upDownControl = forTeam(RED_UP_DOWN_CONTROL);
        upGardenControl = forTeam(RED_UP_GARDEN_CONTROL);
        downFlowerUpControl = forTeam(RED_DOWN_FLOWER_UP_CONTROL);
        downParkControl = forTeam(RED_DOWN_PARK_CONTROL);
        upFlowerDownControl = forTeam(RED_UP_FLOWER_DOWN_CONTROL);
        search1 = forTeam(RED_SEARCH_1);
        search2 = forTeam(RED_SEARCH_2);
        search3 = forTeam(RED_SEARCH_3);
        searchPoses = new double[][]{search1, search2, search3};
    }

    public static double getTeamMinX(double wallMargin) {
        return "B".equals(PoseStorage.TEAM) ? FIELD_MID_X + wallMargin : wallMargin;
    }

    public static double getTeamMaxX(double wallMargin) {
        return "B".equals(PoseStorage.TEAM) ? FIELD_SIZE - wallMargin : FIELD_MID_X - wallMargin;
    }

    public static double getTeamMinY(double wallMargin) {
        return wallMargin;
    }

    public static double getTeamMaxY(double wallMargin) {
        return FIELD_SIZE - wallMargin;
    }

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
        if (Arrays.equals(pose, search1)) return "search1";
        if (Arrays.equals(pose, search2)) return "search2";
        if (Arrays.equals(pose, search3)) return "search3";

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
