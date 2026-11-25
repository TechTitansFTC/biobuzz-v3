package org.firstinspires.ftc.teamcode.game.tests.subsystemTests;

import static org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand.FLOWER_SERVO_UP;
import static org.firstinspires.ftc.teamcode.commands.ShootCommand.GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem.CONTROL_HUB_LOGO_FACING_DIRECTION;
import static org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem.CONTROL_HUB_USB_FACING_DIRECTION;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import org.firstinspires.ftc.teamcode.util.PoseStorage;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Intake and Transfer")
public class IntakeAndTransfer extends OpMode {
    MecanumDriveSubsystem mecanumDriveSubsystem;
    IntakeSubsystem intakeSubsystem;
    FlowerSubsystem flowerSubsystem;
    DriveCommand driveCommand;
    IntakeCommand intakeCommand;
    FlowerIntakeCommand flowerIntakeCommand;
    ReverseCommand reverseCommand;
    IMU imu;
    GamepadEx gamepadEx;
    TelemetryManager telemetryManager;
    GateSubsystem gateSubsystem;
    @Override
    public void init () {
        // Objects
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CONTROL_HUB_LOGO_FACING_DIRECTION, CONTROL_HUB_USB_FACING_DIRECTION)));

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        gamepadEx = new GamepadEx(gamepad1);

        // Subsystems
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap,imu,telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetryManager);
        flowerSubsystem = new FlowerSubsystem(hardwareMap, telemetryManager);
        gateSubsystem = new GateSubsystem(hardwareMap, telemetryManager);

        // Commands
        driveCommand = new DriveCommand(gamepadEx, mecanumDriveSubsystem);
        intakeCommand = new IntakeCommand(intakeSubsystem);
        flowerIntakeCommand = new FlowerIntakeCommand(flowerSubsystem);
        reverseCommand = new ReverseCommand(intakeSubsystem);

        // Bindings
        mecanumDriveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> mecanumDriveSubsystem.resetIMU());
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(flowerIntakeCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(reverseCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> PoseStorage.TEAM = "B");
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> PoseStorage.TEAM = "R");
    }

    @Override
    public void start() {
        flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
        gateSubsystem.setGatePosition(GATE_CLOSE_POSITION);
    }

    @Override
    public void loop () {
        CommandScheduler.getInstance().run();
        telemetry.update();
        gamepadEx.readButtons();
    }

    @Override
    public void stop () {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
