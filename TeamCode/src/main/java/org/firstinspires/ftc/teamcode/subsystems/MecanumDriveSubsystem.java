package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class MecanumDriveSubsystem extends SubsystemBase {
    private final IMU imu;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    public static RevHubOrientationOnRobot.LogoFacingDirection CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private final TelemetryManager telemetry;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, IMU imu, TelemetryManager telemetry) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); // port 0 motor control hub
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); // port 3 motor control hub
        backLeft = hardwareMap.get(DcMotor.class, "backLeft"); // port 1 motor control hub
        backRight = hardwareMap.get(DcMotor.class, "backRight"); // port 2 motor control hub

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        this.imu = imu;
        this.telemetry = telemetry;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addLine("");
        telemetry.addLine("Drive");
        telemetry.addData("backLeftMotorPower", getBackLeftMotorPower());
        telemetry.addData("backRightMotorPower", getBackRightMotorPower());
        telemetry.addData("frontLeftMotorPower", getFrontLeftMotorPower());
        telemetry.addData("frontRightMotorPower", getFrontRightMotorPower());
    }

    public void drive(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void robotCentricDrive(double x, double y, double rotation) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double frontLeftPower = (y + x + rotation) / denominator;
        double backLeftPower = (y - x + rotation) / denominator;
        double frontRightPower = (y - x - rotation) / denominator;
        double backRightPower = (y + x - rotation) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public double getBackLeftMotorPower() { return backLeft.getPower(); }
    public double getBackRightMotorPower() { return backRight.getPower(); }
    public double getFrontLeftMotorPower() { return frontLeft.getPower(); }
    public double getFrontRightMotorPower() { return frontRight.getPower(); }

}
