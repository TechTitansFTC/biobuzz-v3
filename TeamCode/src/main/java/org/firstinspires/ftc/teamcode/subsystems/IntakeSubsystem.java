package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class IntakeSubsystem extends SubsystemBase {
    public static boolean intakeMotorReversed = false;
    public static boolean transferMotorReversed = false;
    public static boolean swapDigitalSensorValues = false;
    public static int LIMELIGHT_POLL_RATE_HZ = 100;
    public static int LIMELIGHT_POLLEN_PIPELINE = 0;
    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private final Servo leftIntakePivotServo;
    private final Servo rightIntakePivotServo;
    private final Limelight3A limelight;
    private final TelemetryManager telemetry;
    private final DigitalChannel sensor1;
    private final DigitalChannel sensor2;
    private final DigitalChannel sensor3;
    private final DigitalChannel sensor4;
    public IntakeSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake"); // port 1 motor expansion hub
        transferMotor = hardwareMap.get(DcMotor.class, "transfer"); // port 0 motor expansion hub

        sensor1 = hardwareMap.get(DigitalChannel.class, "sensor1");
        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");
        sensor3 = hardwareMap.get(DigitalChannel.class, "sensor3");
        sensor4 = hardwareMap.get(DigitalChannel.class, "sensor4");

        leftIntakePivotServo = hardwareMap.get(Servo.class, "leftIntakePivot"); //
        rightIntakePivotServo = hardwareMap.get(Servo.class, "rightIntakePivot"); // port 5 servo expansion hub

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(LIMELIGHT_POLL_RATE_HZ);
        limelight.pipelineSwitch(LIMELIGHT_POLLEN_PIPELINE);
        limelight.start();

        if (intakeMotorReversed) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (transferMotorReversed) {
            transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        this.telemetry = telemetry;

        register();
    }

    @Override
    public void periodic() {
        LLResult result = getLatestLimelightResult();
        boolean valid = result != null && result.isValid();

        telemetry.addLine("");
        telemetry.addLine("Intake/Storage");
        telemetry.addData("Intake Motor Power", getMotorPower());
        telemetry.addData("Left Intake Pivot Servo", getLeftIntakePivotServoPosition());
        telemetry.addData("Right Intake Pivot Servo", getRightIntakePivotServoPosition());
        telemetry.addData("Sensor 1", getSensor1());
        telemetry.addData("Sensor 2", getSensor2());
        telemetry.addData("Sensor 3", getSensor3());
        telemetry.addData("Sensor 4", getSensor4());
        telemetry.addData("Pollen Count", getPollenCount());
        telemetry.addData("Limelight Valid", valid);
        telemetry.addData("Limelight tx", valid ? result.getTx() : 0.0);
        telemetry.addData("Limelight ty", valid ? result.getTy() : 0.0);
        telemetry.addData("Limelight ta", valid ? result.getTa() : 0.0);
    }

    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
        transferMotor.setPower(power);
    }

    public boolean getSensor1() {
        return flip(sensor1.getState());
    }

    public boolean getSensor2() {
        return flip(sensor2.getState());
    }

    public boolean getSensor3() {
        return flip(sensor3.getState());
    }

    public boolean getSensor4() {
        return flip(sensor4.getState());
    }

    public boolean flip(boolean variable) {
        if (swapDigitalSensorValues) return !variable;
        return variable;
    }

    public void setRightIntakePivotServoPosition(double position) {
        rightIntakePivotServo.setPosition(position);
    }

    public void setLeftIntakePivotServoPosition(double position) {
        leftIntakePivotServo.setPosition(position);
    }

    public double getRightIntakePivotServoPosition() {
        return rightIntakePivotServo.getPosition();
    }

    public double getLeftIntakePivotServoPosition() {
        return leftIntakePivotServo.getPosition();
    }

    public double getMotorPower() {
        return intakeMotor.getPower();
    }

    public int getPollenCount() {
        int count = 0;
        if (getSensor1()) count++;
        if (getSensor2()) count++;
        if (getSensor3()) count++;
        if (getSensor4()) count++;
        return count;
    }

    public boolean hasPollen() {
        return getPollenCount() > 0;
    }

    public LLResult getLatestLimelightResult() {
        return limelight.getLatestResult();
    }

    public boolean hasValidPollenTarget(double minTa) {
        LLResult result = getLatestLimelightResult();
        return result != null && result.isValid() && result.getTa() >= minTa;
    }

    public void setLimelightPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void stopLimelight() {
        limelight.stop();
    }
}
