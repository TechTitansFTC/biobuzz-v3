package org.firstinspires.ftc.teamcode.util.paths;

import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.down;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.downFlowerUpControl;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.downParkControl;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.flowerDown1;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.flowerUp1;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.garden1;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.park;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.up;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.upDownControl;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.upFlowerDownControl;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.upGardenControl;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.Arrays;

public class PathChainBuilder {
    public static Path buildPathChain(double[] startPosition, double[] endPosition) {
        Pose startPose = new Pose(startPosition[0], startPosition[1], Math.toRadians(startPosition[2]));
        Pose endPose = new Pose(endPosition[0], endPosition[1], Math.toRadians(endPosition[2]));

        return Paths.line(startPose, endPose).linear(startPose, endPose);
    }

    public static Path buildPathChain(double[] startPosition, double[] endPosition, double velocityConstraint) {
        Pose startPose = new Pose(startPosition[0], startPosition[1], Math.toRadians(startPosition[2]));
        Pose endPose = new Pose(endPosition[0], endPosition[1], Math.toRadians(endPosition[2]));

        return Paths.line(startPose, endPose)
                .linear(startPose, endPose);
//                .with(
//                        Constants.foresightConfig.maxPathSpeed.at(velocityConstraint));
    }

    public static Path buildPathChain(double[] startPosition, double[] endPosition, double[] controlPoint) {
        Pose startPose = new Pose(startPosition[0], startPosition[1], Math.toRadians(startPosition[2]));
        Pose endPose = new Pose(endPosition[0], endPosition[1], Math.toRadians(endPosition[2]));
        Pose control = new Pose(controlPoint[0], controlPoint[1], Math.toRadians(controlPoint[2]));

        return Paths.curve(startPose, control, endPose).linear(startPose, endPose);
    }

    public static Path buildSafePath(double[] startPosition, double[] endPosition) {
        double[] controlPoint = getControlPoint(startPosition, endPosition);
        if (controlPoint != null) {
            return buildPathChain(startPosition, endPosition, controlPoint);
        }
        return buildPathChain(startPosition, endPosition);
    }

    private static double[] getControlPoint(double[] startPosition, double[] endPosition) {
        if (Arrays.equals(startPosition, down) && Arrays.equals(endPosition, park)) {
            return downParkControl;
        }
        if (Arrays.equals(startPosition, up) && Arrays.equals(endPosition, garden1)) {
            return upGardenControl;
        }
        if (Arrays.equals(startPosition, down) && Arrays.equals(endPosition, flowerUp1)) {
            return downFlowerUpControl;
        }
        if (Arrays.equals(startPosition, up) && Arrays.equals(endPosition, flowerDown1)) {
            return upFlowerDownControl;
        }
        if ((Arrays.equals(startPosition, down) && Arrays.equals(endPosition, up))
                || (Arrays.equals(startPosition, up) && Arrays.equals(endPosition, down))) {
            return upDownControl;
        }
        return null;
    }
}
