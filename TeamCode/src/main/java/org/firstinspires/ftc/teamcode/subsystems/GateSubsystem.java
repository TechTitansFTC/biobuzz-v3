package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;


public class GateSubsystem extends SubsystemBase {
    private final Servo servo;
    private final TelemetryManager telemetry;

    public GateSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        servo = hardwareMap.get(Servo.class, "gate");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addLine("");
        telemetry.addLine("Gate");
        telemetry.addData("Gate Position", getGatePosition());
    }

    public void setGatePosition(double position) {
        servo.setPosition(position);
    }

    public double getGatePosition() {
        return servo.getPosition();
    }
}
