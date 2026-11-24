package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_APRIL_TAG_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_IMU_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KD;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KF;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KI;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KD;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KF;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KI;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTING_TIME;

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

    private final ScoringSubsystem scoringSubsystem;
    private final Follower follower;
    private final PIDController pollenTurretServoController;
    private final PIDController pollenTurretMotorController;
    private final ElapsedTime controllerTimer;
    private final ElapsedTime bearingRetainerTimer;

    private double pollenTurretServoPower;
    private double pollenTurretMotorPower;
    private double lateralDistance;
    private double previousBearing;

    public ScoringCommand(ScoringSubsystem scoringSubsystem, Follower follower) {
        this.scoringSubsystem = scoringSubsystem;
        this.follower = follower;

        controllerTimer = new ElapsedTime();
        bearingRetainerTimer = new ElapsedTime();

        pollenTurretServoPower = 0;
        pollenTurretMotorPower = 0;

        lateralDistance = 0;
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

        // TODO: finish target logic like choosing which one
        double targetY = 0;
        double targetX = 57;



        double xDiff = targetX - currX;
        double yDiff = targetY - currY;

        double angle = Math.toDegrees(Math.atan2(xDiff, yDiff));
        angle = normalizeDegrees(angle - currHeading);

        lateralDistance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        double pollenTurretTargetRPM = POLLEN_TURRET_VELOCITY_MODEL(lateralDistance);

        pollenTurretServoController.setTarget(angle);
        pollenTurretMotorController.setTarget(pollenTurretTargetRPM);
        controllerTimer.reset();
        stateTimer.reset();

        scoringState = ScoringRoutine.ALIGN_IMU;
        double currentValuePollenTurretServo = scoringSubsystem.getPollenTurretAngleDegrees();

        pollenTurretServoPower = pollenTurretServoController.calculateOutput(currentValuePollenTurretServo, controllerTimer.seconds());

        if (stateTimer.seconds() >= ALIGN_IMU_TIME) {
            pollenTurretServoController.setTarget(0);
            stateTimer.reset();
            scoringState = ScoringRoutine.ALIGN_APRIL_TAGS;
        }


        double offsetDegreesPollen = scoringSubsystem.getBearing();

        pollenTurretServoPower = pollenTurretServoController.calculateOutput(offsetDegreesPollen, controllerTimer.seconds());

        if (stateTimer.seconds() >= ALIGN_APRIL_TAG_TIME) {
            stateTimer.reset();
            storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_OPEN_POSITION);
            storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
            scoringState = ScoringRoutine.SHOOT;
        }

        double offsetDegreesPollen2 = scoringSubsystem.getBearing();

        pollenTurretServoPower = pollenTurretServoController.calculateOutput(offsetDegreesPollen2, controllerTimer.seconds());

        if (stateTimer.seconds() >= SHOOTING_TIME) {
            scoringState = ScoringRoutine.END;
            stateTimer.reset();
        }

        storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
        storageSubsystem.setTransferMotorPower(0);


        double currentValuePollenTurretMotor = scoringSubsystem.getPollenTurretMotorRawRPM();
        double currentValueNectarTurretMotor = scoringSubsystem.getNectarTurretMotorRPM();


        pollenTurretMotorPower = pollenTurretMotorController.calculateOutput(currentValuePollenTurretMotor, controllerTimer.seconds());

        scoringSubsystem.setPollenTurretServoPower(pollenTurretServoPower);
        scoringSubsystem.setPollenTurretShooterPower(pollenTurretMotorPower);
        controllerTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return endNow || scoringState == ScoringRoutine.END;
    }

    @Override
    public void end(boolean interrupted) {
        storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
        storageSubsystem.setTransferMotorPower(0);
        scoringSubsystem.setPollenTurretServoPower(0);
        scoringSubsystem.setNectarTurretServoPower(0);
    }





    public double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }
}
