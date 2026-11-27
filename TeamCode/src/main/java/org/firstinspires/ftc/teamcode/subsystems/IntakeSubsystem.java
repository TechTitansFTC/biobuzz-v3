package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
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
    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private final Servo leftIntakePivotServo;
    private final Servo rightIntakePivotServo;
    private final TelemetryManager telemetry;
    private final DigitalChannel sensor1;
    private final DigitalChannel sensor2;
    private final DigitalChannel sensor3;
    private final DigitalChannel sensor4;
    public IntakeSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake"); // port 1 motor expansion hub
        transferMotor = hardwareMap.get(DcMotor.class, "transfer"); // port 0 motor expansion hub

        sensor1 = hardwareMap.get(DigitalChannel.class, "sensor1"); // port 0 digital expansion hub
        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2"); // port 2 digital expansion hub
        sensor3 = hardwareMap.get(DigitalChannel.class, "sensor3"); // port 4 digital expansion hub
        sensor4 = hardwareMap.get(DigitalChannel.class, "sensor4"); // port 6 digital expansion hub

        leftIntakePivotServo = hardwareMap.get(Servo.class, "leftIntakePivot"); //
        rightIntakePivotServo = hardwareMap.get(Servo.class, "rightIntakePivot"); // port 5 servo expansion hub

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
        if (telemetry == null) return;

        telemetry.addLine("");
        telemetry.addLine("Intake/Storage");
        telemetry.addData("Intake Motor Power", getMotorPower());
        telemetry.addData("Left Intake Pivot Servo", getLeftIntakePivotServoPosition());
        telemetry.addData("Right Intake Pivot Servo", getRightIntakePivotServoPosition());
        telemetry.addData("Sensor 1", getSensor1());
        telemetry.addData("Sensor 2", getSensor2());
        telemetry.addData("Sensor 3", getSensor3());
        telemetry.addData("Sensor 4", getSensor4());
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
        if (intakeMotor == null) return -1;
        return intakeMotor.getPower();
    }
}
