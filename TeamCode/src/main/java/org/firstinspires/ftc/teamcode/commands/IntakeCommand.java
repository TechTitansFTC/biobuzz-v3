package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_CLOSE_POSITION;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Configurable
public class IntakeCommand extends CommandBase {
    public static double INTAKE_MOTOR_POWER = 1;
    public static double REVERSE_INTAKE_MOTOR_POWER = 1;
    public static double OUT_TAKE_TIME = 0.1;
    public static double FLOWER_RIGHT_SERVO_DOWN = 0.37;
    public static double FLOWER_RIGHT_SERVO_UP = 0.74;
    public static double FLOWER_LEFT_SERVO_DOWN = 0.37 ;
    public static double FLOWER_LEFT_SERVO_UP = 0.64;
    private final StorageSubsystem storageSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private boolean outTaking = false;
    private final ElapsedTime outTakingTimer;

    public IntakeCommand(StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        outTakingTimer = new ElapsedTime();

        addRequirements(storageSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        outTaking = false;
        outTakingTimer.reset();
    }

    @Override
    public void execute() {
        storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
        if (outTaking) {
            if (outTakingTimer.seconds() >= OUT_TAKE_TIME) {
                outTaking = false;
            }
            return;
        }
        int count = 0;
        storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
        if (storageSubsystem.getSensor1Color() != StorageSubsystem.GamePieceColor.NONE) count++;
        if ((storageSubsystem.getSensor1Color() == StorageSubsystem.GamePieceColor.RED) || (storageSubsystem.getSensor1Color() == StorageSubsystem.GamePieceColor.BLUE)) {
            intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
            storageSubsystem.setTransferMotorPower(-REVERSE_TRANSFER_MOTOR_POWER);
            outTakingTimer.reset();
            outTaking = true;
            return;
        }
        if (storageSubsystem.getSensor2Value()) count++;
        if (storageSubsystem.getSensor3Color() != StorageSubsystem.GamePieceColor.NONE) count++;
        if (storageSubsystem.getSensor4Value()) count++;
        if (storageSubsystem.getSensor5Value()) count++;
        if (count < 4) {
            intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
        } else if (count == 4) {
            intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
            storageSubsystem.setTransferMotorPower(0);
        } else {
            outTakingTimer.reset();
            intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
            storageSubsystem.setTransferMotorPower(-REVERSE_TRANSFER_MOTOR_POWER);
            outTaking = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotorPower(0);
        storageSubsystem.setTransferMotorPower(0);
    }
}
