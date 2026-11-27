package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import static org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem.ENCODER_TO_TURRET_RATIO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="Encoder Test", group="Tests")
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput pollenTurretEncoder = hardwareMap.get(AnalogInput.class, "pollenEncoder");

        waitForStart();
        while (opModeIsActive()) {
            double voltage = pollenTurretEncoder.getVoltage();
            double rawDegrees = (voltage / 3.3) * 360;
            double degrees = getPollenTurretAngleDegrees(pollenTurretEncoder);
            telemetry.addData("Raw Degrees", rawDegrees);
            telemetry.addData("Degrees", degrees);
            telemetry.update();
        }
    }

    public double getPollenTurretAngleDegrees(AnalogInput pollenTurretEncoder) {
        double voltage = pollenTurretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // FORWARD WILL BE POSITIVE 180 degrees pleeeeeaase
        rawDegrees -= 180;

        return -rawDegrees * ENCODER_TO_TURRET_RATIO;
    }
}
