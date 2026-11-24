package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.PoseStorage.TEAM;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
public class ScoringSubsystem extends SubsystemBase {
    public static boolean RIGHT_POLLEN_TURRET_MOTOR_REVERSED = false;
    public static boolean LEFT_POLLEN_TURRET_MOTOR_REVERSED = true;
    public static boolean POLLEN_TURRET_CAMERA_REVERSED = true;
    public static double POLLEN_SHOOTER_RPM_FILTER = 0.3;
    public static double ENCODER_TO_TURRET_RATIO = 2.75;

    private final DcMotorEx rightPollenTurretMotor;
    private final DcMotorEx leftPollenTurretMotor;
    private final CRServo pollenTurretServo;
    private final AnalogInput pollenTurretEncoder;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal pollenTurretCamera;
    private final VoltageSensor battery;
    private final TelemetryManager telemetry;
    private double filteredRpm = 0;
    private boolean firstShooterUpdate = true;
    private double targetMotorRPM = 0;
    private double currX;
    private double currY;
    private double currHeading;
    private double angleToTarget;
    private double distanceToTarget;

    public ScoringSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        rightPollenTurretMotor = hardwareMap.get(DcMotorEx.class, "rightPollenShooter");
        leftPollenTurretMotor = hardwareMap.get(DcMotorEx.class, "leftPollenShooter");
        pollenTurretServo = hardwareMap.get(CRServo.class, "pollenTurret");

        pollenTurretEncoder = hardwareMap.get(AnalogInput.class, "pollenEncoder");

        rightPollenTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftPollenTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPollenTurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftPollenTurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setPosition(PoseStorage.autonomousEndPose);

        angleToTarget = 0;
        distanceToTarget = 0;
        currX = 0;
        currY = 0;

        if (RIGHT_POLLEN_TURRET_MOTOR_REVERSED) {
            rightPollenTurretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (LEFT_POLLEN_TURRET_MOTOR_REVERSED) {
            leftPollenTurretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        battery = hardwareMap.voltageSensor.iterator().next();
        aprilTag = new AprilTagProcessor.Builder().setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();

        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        register();
    }

    @Override
    public void periodic() {
        updateFilteredRPM();

        telemetry.addLine("");
        telemetry.addLine("Scoring");
        telemetry.addData("Camera bearing", getBearing());
        telemetry.addData("Pollen shooter RPM", getPollenShooterRPM());
        telemetry.addData("Pollen shooter target", getPollenShooterTargetRPM());
        telemetry.addData("CurrX", getCurrX());
        telemetry.addData("CurrY", getCurrY());
        telemetry.addData("CurrHeading", getCurrHeading());
        telemetry.addData("Angle to Target", getAngleToTarget());
        telemetry.addData("Distance to Target", getDistanceToTarget());
    }

    public void setPollenShooterTargetRPM(double target) { targetMotorRPM = target; }
    public double getPollenShooterTargetRPM() { return targetMotorRPM; }
    public double getPollenShooterRPM() {
        return filteredRpm;
    }

    public void setPollenTurretShooterPower(double power) {
        rightPollenTurretMotor.setPower(power);
    }

    public void setPollenTurretServoPower(double power) {
        pollenTurretServo.setPower(power);
    }

    private double getPollenTurretMotorRawRPM() {
        double ticksPerSecond = rightPollenTurretMotor.getVelocity();

        return (ticksPerSecond * 60) / 28.0;
    }

    public double getBatteryVoltage() {
        return battery.getVoltage();
    }

    public Double getBearing() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata != null) {

                    if (!clusterDet.metadata.name.startsWith(TEAM)) {
                        continue;
                    }

                    if (POLLEN_TURRET_CAMERA_REVERSED) return -clusterDet.ftcPose.bearing;
                    return clusterDet.ftcPose.bearing;
                }
            }
        }

        return 361.0;
    }

    public double getPollenTurretAngleDegrees() {
        double voltage = pollenTurretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // FORWARD WILL BE POSITIVE 180 degrees pleeeeeaase
        rawDegrees -= 180;

        return rawDegrees * ENCODER_TO_TURRET_RATIO;
    }

    private void updateFilteredRPM() {
        if (firstShooterUpdate) {
            filteredRpm = getPollenTurretMotorRawRPM();
            firstShooterUpdate = false;
        }
        filteredRpm += Range.clip(POLLEN_SHOOTER_RPM_FILTER, 0.01, 1) * (getPollenTurretMotorRawRPM() - filteredRpm);
    }

    public void setPosition(Pose position) { // x, y, heading
        currX = position.x();
        currY = position.y();
        currHeading = convertHeading(Math.toDegrees(position.heading()));
    }

    public double getCurrX() {
        return currX;
    }

    public double getCurrY() {
        return currY;
    }

    public double getCurrHeading() {
        return currHeading;
    }

    private double convertHeading(double beforeDegrees) {
        double after = 90.0 - beforeDegrees;

        while (after > 180.0) {
            after -= 360.0;
        }
        while (after <= -180.0) {
            after += 360.0;
        }

        return after;
    }

    public void setAngleToTarget(double xDiff, double yDiff) {
        angleToTarget = Math.toDegrees(Math.atan2(xDiff, yDiff));
    }

    public double getAngleToTarget() {
        return angleToTarget;
    }

    public void setDistanceToTarget(double xDiff, double yDiff) {
        distanceToTarget = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }
}