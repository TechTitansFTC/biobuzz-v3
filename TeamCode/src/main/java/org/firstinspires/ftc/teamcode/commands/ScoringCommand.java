package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_APRIL_TAG_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_IMU_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTING_TIME;
import static org.firstinspires.ftc.teamcode.util.PoseStorage.TEAM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Configurable
public class ScoringCommand extends CommandBase {
    public static double RETAIN_BEARING_TIME = 0.35; // seconds
    public static double POLLEN_TURRET_SERVO_CONTROLLER_KP = 0.0007;
    public static double POLLEN_TURRET_SERVO_CONTROLLER_KI = 0.0;
    public static double POLLEN_TURRET_SERVO_CONTROLLER_KD = 0.0000004;
    public static double POLLEN_TURRET_SERVO_CONTROLLER_KF = 0.06;
    public static double POLLEN_SHOOTER_SERVO_CONTROLLER_KP = 0.001;
    public static double POLLEN_SHOOTER_SERVO_CONTROLLER_KI = 0.0;
    public static double POLLEN_SHOOTER_SERVO_CONTROLLER_KD = 0.004;
    public static double POLLEN_SHOOTER_SERVO_CONTROLLER_KF = 0.06;
    public static double POLLEN_SHOOTER_VELOCITY_MODEL_A = 0.1;
    public static double POLLEN_SHOOTER_VELOCITY_MODEL_B = 1.1;
    public static double POLLEN_SHOOTER_VELOCITY_MODEL_K = 2000;
    public static double TARGET_X_RED = 55.5;
    public static double TARGET_X_BLUE = 84;
    public static double TARGET_Y_DOWN = 55;
    public static double TARGET_Y_UP = 85.5;
    public static double FIELD_MID_Y = 72;

    private final ScoringSubsystem scoringSubsystem;
    private final Follower follower;
    private final PIDController pollenTurretServoController;
    private final PIDController pollenTurretMotorController;
    private final ElapsedTime controllerTimer;
    private final ElapsedTime bearingRetainerTimer;

    private double pollenTurretServoPower;
    private double pollenTurretMotorPower;
    private double previousBearing;

    public ScoringCommand(ScoringSubsystem scoringSubsystem, Follower follower) {
        this.scoringSubsystem = scoringSubsystem;
        this.follower = follower;

        controllerTimer = new ElapsedTime();
        bearingRetainerTimer = new ElapsedTime();

        pollenTurretServoPower = 0;
        pollenTurretMotorPower = 0;

        previousBearing = 0;

        pollenTurretServoController = new PIDController(POLLEN_TURRET_SERVO_CONTROLLER_KP, POLLEN_TURRET_SERVO_CONTROLLER_KI,
                POLLEN_TURRET_SERVO_CONTROLLER_KD, POLLEN_TURRET_SERVO_CONTROLLER_KF);
        pollenTurretMotorController = new PIDController(POLLEN_SHOOTER_SERVO_CONTROLLER_KP, POLLEN_SHOOTER_SERVO_CONTROLLER_KI,
                POLLEN_SHOOTER_SERVO_CONTROLLER_KD, POLLEN_SHOOTER_SERVO_CONTROLLER_KF);

        addRequirements(scoringSubsystem);
    }

    @Override
    public void initialize() {
        controllerTimer.reset();
        bearingRetainerTimer.reset();
    }

    @Override
    public void execute() {
        pollenTurretServoController.setConstants(POLLEN_TURRET_SERVO_CONTROLLER_KP, POLLEN_TURRET_SERVO_CONTROLLER_KI,
                POLLEN_TURRET_SERVO_CONTROLLER_KD, POLLEN_TURRET_SERVO_CONTROLLER_KF);
        pollenTurretMotorController.setConstants(POLLEN_SHOOTER_SERVO_CONTROLLER_KP, POLLEN_SHOOTER_SERVO_CONTROLLER_KI,
                POLLEN_SHOOTER_SERVO_CONTROLLER_KD, POLLEN_SHOOTER_SERVO_CONTROLLER_KF);

        scoringSubsystem.setPosition(follower.pose());
        double currX = scoringSubsystem.getCurrX();
        double currY = scoringSubsystem.getCurrY();
        double currHeading = scoringSubsystem.getCurrHeading();

        // TODO: finish hood logic
        double targetX;
        double targetY;
        if (TEAM.equals("R")) {
            targetX = TARGET_X_RED;
        } else {
            targetX = TARGET_X_BLUE;
        }
        if (currY >= FIELD_MID_Y) {
            targetY = TARGET_Y_UP;
        } else {
            targetY = TARGET_Y_DOWN;
        }

        double xDiff = targetX - currX;
        double yDiff = targetY - currY;

        // Motor Power
        scoringSubsystem.setDistanceToTarget(xDiff, yDiff);

        double distanceToTarget = scoringSubsystem.getDistanceToTarget();
        double pollenTurretTargetRPM = POLLEN_TURRET_VELOCITY_MODEL(distanceToTarget);
        double pollenTurretCurrentRPM = scoringSubsystem.getPollenShooterRPM();
        scoringSubsystem.setPollenShooterTargetRPM(pollenTurretTargetRPM);
        pollenTurretMotorController.setTarget(pollenTurretTargetRPM);
        pollenTurretMotorPower = pollenTurretMotorController.calculateOutput(pollenTurretCurrentRPM, controllerTimer.seconds());

        double hoodTargetPos = HOOD_TURRET_POSITION_MODEL(distanceToTarget);

        scoringSubsystem.setHoodPosition(hoodTargetPos);

        // Servo Power
        double offsetDegreesPollen = scoringSubsystem.getBearing();
        scoringSubsystem.setAngleToTarget(xDiff, yDiff);
        if (offsetDegreesPollen != 361 || bearingRetainerTimer.seconds() <= RETAIN_BEARING_TIME) {
            if (offsetDegreesPollen != 361) {
                previousBearing = offsetDegreesPollen;
                bearingRetainerTimer.reset();
            }
            pollenTurretServoController.setTarget(0);
            pollenTurretServoPower = pollenTurretServoController.calculateOutput(previousBearing, controllerTimer.seconds());
        } else {
            double angleToTarget = scoringSubsystem.getAngleToTarget();
            double currentValuePollenTurretServo = scoringSubsystem.getPollenTurretAngleDegrees();
            pollenTurretServoController.setTarget(angleToTarget);
            pollenTurretServoPower = pollenTurretServoController.calculateOutput(currentValuePollenTurretServo, controllerTimer.seconds());
        }

        scoringSubsystem.setPollenTurretServoPower(pollenTurretServoPower);
        scoringSubsystem.setPollenTurretShooterPower(pollenTurretMotorPower);
        controllerTimer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        scoringSubsystem.setPollenTurretServoPower(0);
        scoringSubsystem.setPollenTurretShooterPower(0);
    }


    private double POLLEN_TURRET_VELOCITY_MODEL(double distanceToTarget) {
        return POLLEN_SHOOTER_VELOCITY_MODEL_A * Math.pow(POLLEN_SHOOTER_VELOCITY_MODEL_B, distanceToTarget) +
                POLLEN_SHOOTER_VELOCITY_MODEL_K;
    }
    private double HOOD_TURRET_POSITION_MODEL(double distanceToTarget) {
        return POLLEN_SHOOTER_VELOCITY_MODEL_A * Math.pow(POLLEN_SHOOTER_VELOCITY_MODEL_B, distanceToTarget) +
                POLLEN_SHOOTER_VELOCITY_MODEL_K;
    }
}
