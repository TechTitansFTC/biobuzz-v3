package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.getTeamMaxX;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.getTeamMaxY;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.getTeamMinX;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.getTeamMinY;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.searchPoses;
import static org.firstinspires.ftc.teamcode.util.paths.PathChainBuilder.buildPathChain;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Configurable
public class GroundIntakeController {
    public enum PatrolResult {
        SEARCHING,
        TARGET_LOCKED
    }

    public enum ChaseResult {
        CHASING,
        READY_TO_SHOOT,
        NEEDS_BUFFER
    }

    public static double GROUND_INTAKE_TIMEOUT = 2.5;
    public static double GROUND_INTAKE_TURN_KP = 0.02;
    public static double GROUND_INTAKE_TURN_KI = 0.0;
    public static double GROUND_INTAKE_TURN_KD = 0.0;
    public static double GROUND_INTAKE_TURN_KF = 0.0;
    public static double GROUND_INTAKE_FORWARD_POWER = 0.35;
    public static double GROUND_INTAKE_SCAN_TURN_POWER = 0.25;
    public static double MIN_POLLEN_TA = 0.2;
    public static int POLLEN_CONFIRM_FRAMES = 3;
    public static double PATROL_VELOCITY_CONSTRAINT = 0.65;
    public static double GROUND_INTAKE_WALL_MARGIN = 10.0;

    private final Follower follower;
    private final IntakeSubsystem intakeSubsystem;
    private final PIDController turnController;
    private final ElapsedTime turnTimer;
    private final ElapsedTime chaseTimer;

    private int searchIndex = 0;
    private int validFrames = 0;
    private int lastPollenCount = 0;

    public GroundIntakeController(Follower follower, IntakeSubsystem intakeSubsystem) {
        this.follower = follower;
        this.intakeSubsystem = intakeSubsystem;

        this.turnController = new PIDController(
                GROUND_INTAKE_TURN_KP,
                GROUND_INTAKE_TURN_KI,
                GROUND_INTAKE_TURN_KD,
                GROUND_INTAKE_TURN_KF
        );
        this.turnController.setTarget(0);

        this.turnTimer = new ElapsedTime();
        this.chaseTimer = new ElapsedTime();
    }

    public double[] startPatrol(double[] currPos) {
        validFrames = 0;
        double[] target = searchPoses[searchIndex];
        follower.follow(buildPathChain(currPos, target, PATROL_VELOCITY_CONSTRAINT));
        return target;
    }

    public PatrolResult updatePatrol(double[] currentLivePose) {
        if (intakeSubsystem.hasValidPollenTarget(MIN_POLLEN_TA)) {
            validFrames++;
        } else {
            validFrames = 0;
        }

        if (validFrames >= POLLEN_CONFIRM_FRAMES) {
            validFrames = 0;
            follower.stop();
            turnController.reset();
            turnTimer.reset();
            chaseTimer.reset();
            lastPollenCount = intakeSubsystem.getPollenCount();
            return PatrolResult.TARGET_LOCKED;
        }

        if (!follower.isBusy()) {
            searchIndex = (searchIndex + 1) % searchPoses.length;
            follower.follow(buildPathChain(currentLivePose, searchPoses[searchIndex], PATROL_VELOCITY_CONSTRAINT));
        }

        return PatrolResult.SEARCHING;
    }

    public ChaseResult updateChase(int currentPollenCount, int minPollenToShoot) {
        if (currentPollenCount > lastPollenCount) {
            lastPollenCount = currentPollenCount;
            chaseTimer.reset();
        }

        if (currentPollenCount >= minPollenToShoot) {
            follower.stop();
            return ChaseResult.READY_TO_SHOOT;
        }

        if (isOutOfBounds(follower.pose()) || chaseTimer.seconds() >= GROUND_INTAKE_TIMEOUT) {
            follower.stop();
            searchIndex = (searchIndex + 1) % searchPoses.length;
            return ChaseResult.NEEDS_BUFFER;
        }

        driveTowardPollen();
        return ChaseResult.CHASING;
    }

    public double[] getCurrentSearchTarget() {
        return searchPoses[searchIndex];
    }

    private boolean isOutOfBounds(Pose pose) {
        return pose.x() < getTeamMinX(GROUND_INTAKE_WALL_MARGIN)
                || pose.x() > getTeamMaxX(GROUND_INTAKE_WALL_MARGIN)
                || pose.y() < getTeamMinY(GROUND_INTAKE_WALL_MARGIN)
                || pose.y() > getTeamMaxY(GROUND_INTAKE_WALL_MARGIN);
    }

    private void driveTowardPollen() {
        turnController.setConstants(
                GROUND_INTAKE_TURN_KP,
                GROUND_INTAKE_TURN_KI,
                GROUND_INTAKE_TURN_KD,
                GROUND_INTAKE_TURN_KF
        );

        LLResult result = intakeSubsystem.getLatestLimelightResult();
        if (result != null && result.isValid()) {
            double turn = Range.clip(
                    turnController.calculateOutput(result.getTx(), turnTimer.seconds()),
                    -1.0,
                    1.0
            );
            turnTimer.reset();
            follower.manual(GROUND_INTAKE_FORWARD_POWER, 0, turn);
        } else {
            turnController.reset();
            turnTimer.reset();
            double scanTurn = "B".equals(PoseStorage.TEAM)
                    ? -GROUND_INTAKE_SCAN_TURN_POWER
                    : GROUND_INTAKE_SCAN_TURN_POWER;
            follower.manual(0, 0, scanTurn);
        }
    }
}
