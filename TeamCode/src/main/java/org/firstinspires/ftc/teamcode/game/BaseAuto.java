package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand.FLOWER_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand.FLOWER_SERVO_UP;
import static org.firstinspires.ftc.teamcode.commands.IntakeCommand.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.commands.ShootCommand.GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.*;
import static org.firstinspires.ftc.teamcode.util.paths.PathChainBuilder.buildSafePath;

import java.util.Arrays;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ScoringCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.util.GroundIntakeController;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Configurable
public abstract class BaseAuto extends OpMode {
    public static double PARK_TIME = 5;
    public static double SHOOTER_SPIN_UP_TIME = 0;
    public static double AUTO_INTAKE_TIME = 1;
    public static double APRIL_TAG_ALIGN_DETECTION_TIME = 1;
    public static int MIN_POLLEN_TO_SHOOT = 3;
    public static double POLLEN_DETECT_BUFFER_TIME = 0.5;

    private enum AutoState {
        START,
        IDLE,
        SHOOT,
        INTAKE_1,
        INTAKE_2,
        POLLEN_BUFFER,
        MOVING,
        CHECK_TIP,
        SEARCH_POLLEN,
        GROUND_INTAKE,
        PARK
    }

    private final String team;

    private IntakeSubsystem intakeSubsystem;
    private ScoringSubsystem scoringSubsystem;
    private FlowerSubsystem flowerSubsystem;
    private GateSubsystem gateSubsystem;
    private IntakeCommand intakeCommand;
    private ScoringCommand scoringCommand;
    private ShootCommand shootCommand;

    private Follower follower;
    private GroundIntakeController groundIntakeController;
    private TelemetryManager telemetryManager;

    private ElapsedTime autoTimer;
    private ElapsedTime aprilTagTimer;
    private ElapsedTime pollenDetectTimer;

    private AutoState currentState;
    private double[] currPos;
    private boolean isShooting = false;
    private boolean tip = false; // false = down, true = up
    private boolean knowTip = false;
    private boolean flowerUp = true; // true = contains pollen, false = already intaked from
    private boolean flowerDown = true;
    private boolean garden = true;
    private int hasPollen = 0;
    private double intakeTime = 0;

    protected BaseAuto(String team) {
        this.team = team;
    }

    @Override
    public void init() {
        PoseStorage.TEAM = team;
        updateForTeam();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.create(hardwareMap);
        follower.setX(initial[0]);
        follower.setY(initial[1]);
        follower.setHeading(Math.toRadians(initial[2]));
        currPos = initial;

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetryManager);
        scoringSubsystem = new ScoringSubsystem(hardwareMap, telemetryManager);
        flowerSubsystem = new FlowerSubsystem(hardwareMap, telemetryManager);
        gateSubsystem = new GateSubsystem(hardwareMap, telemetryManager);

        scoringCommand = new ScoringCommand(scoringSubsystem, follower);
        intakeCommand = new IntakeCommand(intakeSubsystem);
        shootCommand = new ShootCommand(gateSubsystem);

        groundIntakeController = new GroundIntakeController(follower, intakeSubsystem);

        autoTimer = new ElapsedTime();
        aprilTagTimer = new ElapsedTime();
        pollenDetectTimer = new ElapsedTime();

        intakeCommand.schedule();
        scoringCommand.schedule();
    }

    @Override
    public void start() {
        currentState = AutoState.START;
        flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
        gateSubsystem.setGatePosition(GATE_CLOSE_POSITION);
        autoTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();
        PoseStorage.autonomousEndPose = follower.pose();
        hasPollen = intakeSubsystem.getPollenCount();

        if (shouldInterruptForPark()) {
            triggerPark();
            updateTelemetry();
            return;
        }

        switch (currentState) {
            case START:
                handleStart();
                break;
            case IDLE:
                handleIdle();
                break;
            case SHOOT:
                handleShoot();
                break;
            case INTAKE_1:
                handleIntake1();
                break;
            case INTAKE_2:
                handleIntake2();
                break;
            case POLLEN_BUFFER:
                handlePollenBuffer();
                break;
            case MOVING:
                handleMoving();
                break;
            case CHECK_TIP:
                handleCheckTip();
                break;
            case SEARCH_POLLEN:
                handleSearchPollen();
                break;
            case GROUND_INTAKE:
                handleGroundIntake();
                break;
            case PARK:
                if (handlePark()) return;
                break;
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        follower.stop();
        intakeSubsystem.stopLimelight();
        CommandScheduler.getInstance().reset();
    }

    private void handleStart() {
        moveTo(down);
    }

    private void handleIdle() {
        if (isAtIntakeStartPose()) {
            currentState = AutoState.INTAKE_1;
            return;
        }

        if (hasPollen >= MIN_POLLEN_TO_SHOOT) {
            handleReadyToScore();
            return;
        }

        if (garden) {
            garden = false;
            moveTo(garden1);
            return;
        }

        if (flowerUp) {
            flowerUp = false;
            moveTo(flowerUp1);
            return;
        }

        if (flowerDown) {
            flowerDown = false;
            moveTo(flowerDown1);
            return;
        }

        currPos = groundIntakeController.startPatrol(currPos);
        currentState = AutoState.SEARCH_POLLEN;
    }

    private void handleReadyToScore() {
        if (knowTip) {
            currentState = AutoState.SHOOT;
            return;
        }

        if (Arrays.equals(currPos, down) || Arrays.equals(currPos, up)) {
            aprilTagTimer.reset();
            currentState = AutoState.CHECK_TIP;
            return;
        }

        boolean goUp = Arrays.equals(currPos, flowerUp1)
                || Arrays.equals(currPos, flowerUp2)
                || currPos[1] >= FIELD_MID_Y;
        moveTo(goUp ? up : down);
    }

    private void handleShoot() {
        if (autoTimer.seconds() <= SHOOTER_SPIN_UP_TIME) return;

        if (!isShooting) {
            knowTip = false;
            CommandScheduler.getInstance().schedule(shootCommand);
            isShooting = true;
        }

        if (!CommandScheduler.getInstance().isScheduled(shootCommand)) {
            isShooting = false;
            currentState = AutoState.IDLE;
        }
    }

    private void handleIntake1() { // Assumes intake transition time < travel time from pos 1 to pos 2
        intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);

        boolean isFlowerIntake = Arrays.equals(currPos, flowerDown1) || Arrays.equals(currPos, flowerUp1);
        flowerSubsystem.setServoPosition(isFlowerIntake ? FLOWER_SERVO_DOWN : FLOWER_SERVO_UP);

        if (Arrays.equals(currPos, flowerDown1)) {
            moveToInState(flowerDown2, AutoState.INTAKE_2);
        } else if (Arrays.equals(currPos, flowerUp1)) {
            moveToInState(flowerUp2, AutoState.INTAKE_2);
        } else if (Arrays.equals(currPos, garden1)) {
            moveToInState(garden2, AutoState.INTAKE_2);
        }

        intakeTime = autoTimer.seconds();
    }

    private void handleIntake2() {
        if (autoTimer.seconds() - intakeTime < AUTO_INTAKE_TIME) return;

        flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
        if (hasPollen >= MIN_POLLEN_TO_SHOOT) {
            currentState = AutoState.IDLE;
        } else {
            startPollenBuffer();
        }
    }

    private void handlePollenBuffer() {
        if (hasPollen >= MIN_POLLEN_TO_SHOOT || pollenDetectTimer.seconds() >= POLLEN_DETECT_BUFFER_TIME) {
            currentState = AutoState.IDLE;
        }
    }

    private void handleMoving() {
        if (!follower.isBusy()) {
            currentState = AutoState.IDLE;
        }
    }

    private void handleCheckTip() {
        boolean tagVisible = scoringSubsystem.getBearing() != 361.0;

        if (Arrays.equals(currPos, down)) {
            if (tagVisible) {
                tip = false;
                knowTip = true;
                currentState = AutoState.IDLE;
            } else if (aprilTagTimer.seconds() >= APRIL_TAG_ALIGN_DETECTION_TIME) {
                moveTo(up);
            }
        } else if (Arrays.equals(currPos, up)) {
            if (tagVisible) {
                tip = true;
                knowTip = true;
                currentState = AutoState.IDLE;
            } else if (aprilTagTimer.seconds() >= APRIL_TAG_ALIGN_DETECTION_TIME) {
                moveTo(down);
            }
        } else {
            currentState = AutoState.IDLE;
        }
    }

    private void handleSearchPollen() {
        GroundIntakeController.PatrolResult result = groundIntakeController.updatePatrol(getCurrentPoseArray());
        if (result == GroundIntakeController.PatrolResult.TARGET_LOCKED) {
            flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
            intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
            currentState = AutoState.GROUND_INTAKE;
        } else {
            currPos = groundIntakeController.getCurrentSearchTarget();
        }
    }

    private void handleGroundIntake() {
        GroundIntakeController.ChaseResult result = groundIntakeController.updateChase(hasPollen, MIN_POLLEN_TO_SHOOT);
        if (result == GroundIntakeController.ChaseResult.READY_TO_SHOOT) {
            currPos = getCurrentPoseArray();
            currentState = AutoState.IDLE;
        } else if (result == GroundIntakeController.ChaseResult.NEEDS_BUFFER) {
            currPos = getCurrentPoseArray();
            startPollenBuffer();
        }
    }

    private boolean handlePark() {
        if (!follower.isBusy()) {
            requestOpModeStop();
            return true;
        }
        return false;
    }

    private boolean isTimeToPark() {
        return 30 - autoTimer.seconds() <= PARK_TIME;
    }

    private boolean shouldInterruptForPark() {
        return isTimeToPark()
                && currentState != AutoState.PARK
                && currentState != AutoState.START
                && currentState != AutoState.SHOOT;
    }

    private void triggerPark() {
        flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
        double[] startPose = Arrays.equals(currPos, down) && !follower.following()
                ? down
                : getCurrentPoseArray();
        moveToInState(startPose, park, AutoState.PARK);
    }

    private boolean isAtIntakeStartPose() {
        return Arrays.equals(currPos, flowerUp1)
                || Arrays.equals(currPos, flowerDown1)
                || Arrays.equals(currPos, garden1);
    }

    private void startPollenBuffer() {
        pollenDetectTimer.reset();
        currentState = AutoState.POLLEN_BUFFER;
    }

    private void moveTo(double[] target) {
        moveToInState(currPos, target, AutoState.MOVING);
    }

    private void moveToInState(double[] target, AutoState nextState) {
        moveToInState(currPos, target, nextState);
    }

    private void moveToInState(double[] from, double[] target, AutoState nextState) {
        follower.follow(buildSafePath(from, target));
        currPos = target;
        currentState = nextState;
    }

    private double[] getCurrentPoseArray() {
        Pose p = follower.pose();
        return new double[]{p.x(), p.y(), Math.toDegrees(p.heading())};
    }

    private void updateTelemetry() {
        telemetry.addData("Team", PoseStorage.TEAM);
        telemetry.addData("State", currentState);
        telemetry.addData("currPos", getPose(currPos));
        telemetry.addData("hasPollen", hasPollen);
        telemetry.addData("tip", tip);
        telemetry.addData("knowTip", knowTip);
        telemetry.addData("garden", garden);
        telemetry.addData("flowerUp", flowerUp);
        telemetry.addData("flowerDown", flowerDown);
        telemetry.update();
    }
}
