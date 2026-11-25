package org.firstinspires.ftc.teamcode.game;


import static org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand.FLOWER_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand.FLOWER_SERVO_UP;
import static org.firstinspires.ftc.teamcode.commands.IntakeCommand.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.commands.ShootCommand.GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths.*;
import static org.firstinspires.ftc.teamcode.util.paths.PathChainBuilder.buildPathChain;

import java.util.Arrays;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ScoringCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Configurable
@Autonomous(name = "Red Auto", group="Main")
public class RedAuto extends OpMode {
    // TODO: Shoot and go to closest april tag check position without going near the hive repeatedly
    // TODO: Ground intake pollen when done
    public static double PARK_TIME = 5;
    public static double SHOOTER_SPIN_UP_TIME = 0;
    public static double AUTO_INTAKE_TIME = 1;
    public static double APRIL_TAG_ALIGN_DETECTION_TIME = 1;
    IntakeSubsystem intakeSubsystem;
    ScoringSubsystem scoringSubsystem;
    FlowerIntakeCommand flowerIntakeCommand;
    FlowerSubsystem flowerSubsystem;
    GateSubsystem gateSubsystem;
    IntakeCommand intakeCommand;
    ScoringCommand scoringCommand;
    ShootCommand shootCommand;
    private Follower follower;
    private ElapsedTime autoTimer;
    boolean isShooting = false;
    private boolean tip = false; // down, true is up (down means down is correct scoring position, up means up is correct scoring position)
    private boolean knowTip = false;
    private boolean flowerUp = true; // true --> contains balls, false --> already intaked from
    private boolean flowerDown = true;
    private boolean garden = true;
    private boolean hasPollen = true;
    private ElapsedTime aprilTagTimer;
    private double[] currPos;
    private enum AutoState {START, IDLE, SHOOT, INTAKE_1, INTAKE_2, MOVING, CHECK_TIP, PARK}
    private AutoState currentState;
    private boolean doneWithFirstMove = false;
    private boolean hasShot = false;
    int detectedAprilTag = 0;
    private boolean tipCheckedAtAll = false;
    private boolean visited = false;
    private boolean visted2 = false;
    double intakeTime = 0;
    TelemetryManager telemetryManager;
    @Override
    public void init() {
        PoseStorage.TEAM = "R";
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
        flowerIntakeCommand = new FlowerIntakeCommand(flowerSubsystem);
        intakeCommand = new IntakeCommand(intakeSubsystem);
        shootCommand = new ShootCommand(gateSubsystem);

        aprilTagTimer = new ElapsedTime();
        aprilTagTimer.reset();
        autoTimer = new ElapsedTime();
        autoTimer.reset();

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
        switch (currentState) {
            case START:
                follower.follow(buildPathChain(initial, down));
                currentState = AutoState.MOVING;
                currPos = down;
                break;
            case IDLE:
                if (30 - autoTimer.seconds() <= PARK_TIME) {
                    if (Arrays.equals(currPos, down)) {
                        follower.follow(buildPathChain(currPos, park, downParkControl));
                    } else {
                        follower.follow(buildPathChain(currPos, park));
                    }
                    currPos = park;
                    currentState = AutoState.PARK;
                    break;
                }
                if (Arrays.equals(currPos, flowerUp1) || Arrays.equals(currPos, flowerDown1) || Arrays.equals(currPos, garden1)) {
                    currentState = AutoState.INTAKE_1;
                    break;
                }
                if (hasPollen) {
                    visted2 = true;
                    if (knowTip) {
                        hasPollen = false;
                        currentState = AutoState.SHOOT;
                        break;
                    } else {
                        if (Arrays.equals(currPos, down) || Arrays.equals(currPos, up)) {
                            visited = true;
                            aprilTagTimer.reset();
                            currentState = AutoState.CHECK_TIP;
                            break;
                        } else {
                            if (Arrays.equals(currPos, flowerUp1) || Arrays.equals(currPos, flowerUp2)) {
                                follower.follow(buildPathChain(currPos, up));
                                currPos = up;
                            } else {
                                follower.follow(buildPathChain(currPos, down));
                                currPos = down;
                            }
                            currentState = AutoState.MOVING;
                        }
                        break;
                    }
                }
                if (garden) {
                    garden = false;
                    if (Arrays.equals(currPos, up)) {
                        follower.follow(buildPathChain(currPos, garden1, upGardenControl));
                    } else {
                        follower.follow(buildPathChain(currPos, garden1));
                    }
                    currPos = garden1;
                    currentState = AutoState.MOVING;
                    break;
                }
                if (flowerUp) {
                    flowerUp = false;
                    if (Arrays.equals(currPos, down)) {
                        follower.follow(buildPathChain(currPos, flowerUp1, downFlowerUpControl));
                    } else {
                        follower.follow(buildPathChain(currPos, flowerUp1));
                    }
                    currPos = flowerUp1;
                    currentState = AutoState.MOVING;
                    break;
                }
                if (flowerDown) {
                    flowerDown = false;
                    if (Arrays.equals(currPos, up)) {
                        follower.follow(buildPathChain(currPos, flowerDown1, upFlowerDownControl));
                    } else {
                        follower.follow(buildPathChain(currPos, flowerDown1));
                    }
                    currPos = flowerDown1;
                    currentState = AutoState.MOVING;
                    break;
                }
                break;
            case SHOOT:
                if (autoTimer.seconds() <= SHOOTER_SPIN_UP_TIME) break;
                if (!isShooting) {
                    hasPollen = false;
                    knowTip = false;
                    hasShot = true;
                    CommandScheduler.getInstance().schedule(shootCommand);
                    isShooting = true;
                }
                if (!CommandScheduler.getInstance().isScheduled(shootCommand)) {
                    isShooting = false;
                    currentState = AutoState.IDLE;
                }
                break;
            case INTAKE_1: // Assumes that intake transition time < time to go from intake pos 1 to 2
//                intakeSubsystem.setWheelPivotPosition(intakeWheelPivotPositionUp);
//                intakeSubsystem.setRampPivotPosition(intakeRampPivotPositionUp);
                intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
                if (Arrays.equals(currPos, flowerDown1) || Arrays.equals(currPos, flowerUp1)) {
                    flowerSubsystem.setServoPosition(FLOWER_SERVO_DOWN);
//                    flowerSubsystem.setLeftServoPosition(FLOWER_LEFT_SERVO_DOWN);
                } else {
                    flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
//                    flowerSubsystem.setLeftServoPosition(FLOWER_LEFT_SERVO_UP);
                }
                if (Arrays.equals(currPos, flowerDown1)) {
                    follower.follow(buildPathChain(currPos, flowerDown2));
                    currPos = flowerDown2;
                } else if (Arrays.equals(currPos, flowerUp1)) {
                    follower.follow(buildPathChain(currPos, flowerUp2));
                    currPos = flowerUp2;
                } else if (Arrays.equals(currPos, garden1)) {
                    follower.follow(buildPathChain(currPos, garden2));
                    currPos = garden2;
                }
                currentState = AutoState.INTAKE_2;
                intakeTime = autoTimer.seconds();
                break;
            case INTAKE_2:
                if (autoTimer.seconds() - intakeTime >= AUTO_INTAKE_TIME) {
                    hasPollen = true;
                    flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
                    currentState = AutoState.IDLE;
                }
                break;
            case MOVING:
                if (!follower.isBusy()) {
                    currentState = AutoState.IDLE;
                    doneWithFirstMove = true;
                }
                break;
            case CHECK_TIP:
                tipCheckedAtAll = true;
                boolean tagVisible = scoringSubsystem.getBearing() != 361.0;

                if (Arrays.equals(currPos, down)) {
                    if (tagVisible) {
                        tip = false;
                        knowTip = true;
                        currentState = AutoState.IDLE;
                    } else if (aprilTagTimer.seconds() >= APRIL_TAG_ALIGN_DETECTION_TIME) {
                        follower.follow(buildPathChain(currPos, up, upDownControl));
                        currPos = up;
                        currentState = AutoState.MOVING;
                    }
                } else if (Arrays.equals(currPos, up)) {
                    if (tagVisible) {
                        tip = true;
                        knowTip = true;
                        currentState = AutoState.IDLE;
                    } else if (aprilTagTimer.seconds() >= APRIL_TAG_ALIGN_DETECTION_TIME) {
                        follower.follow(buildPathChain(currPos, down, upDownControl));
                        currPos = down;
                        currentState = AutoState.MOVING;
                    }
                } else {
                    currentState = AutoState.IDLE;
                }
                break;
            case PARK:
                if (!follower.isBusy()) {
                    requestOpModeStop();
                    return;
                }
                break;
        }
        PoseStorage.autonomousEndPose = follower.pose();
        telemetry.addData("", "");
        telemetry.addData("State", currentState);
        telemetry.addData("tip", tip);
        telemetry.addData("knowTip", knowTip);
        telemetry.addData("flowerUp", flowerUp);
        telemetry.addData("flowerDown", flowerDown);
        telemetry.addData("garden", garden);
        telemetry.addData("hasPollen", hasPollen);
        telemetry.addData("doneWithFirstMove", doneWithFirstMove);
        telemetry.addData("currPos", getPose(currPos));
        telemetry.addData("lastDetectedapriltag", detectedAprilTag);
        telemetry.addData("hasShot", hasShot);
        telemetry.addData("tipChecked", tipCheckedAtAll);
        telemetry.addData("tipchecked0", visited);
        telemetry.addData("haspollen1", visted2);
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}