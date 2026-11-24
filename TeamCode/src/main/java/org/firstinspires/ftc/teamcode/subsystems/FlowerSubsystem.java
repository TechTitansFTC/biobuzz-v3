package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class FlowerSubsystem extends SubsystemBase {
    private final TelemetryManager telemetry;
    private final Servo servo;

    public FlowerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        servo = hardwareMap.get(Servo.class, "flowerIntake");

        register();
    }

    @Override
    public void periodic() {
        telemetry.addLine("");
        telemetry.addLine("Flower");
        telemetry.addData("flowerServo", getServoPosition());
    }


    public void setServoPosition(double position) {
        servo.setPosition(position);
    }

    public double getServoPosition() {
        return servo.getPosition();
    }
}
