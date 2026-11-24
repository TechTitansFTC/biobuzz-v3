package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ReverseCommand extends CommandBase {
    public static double REVERSE_INTAKE_MOTOR_POWER = 1;

    private final IntakeSubsystem intakeSubsystem;

    public ReverseCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
    }

    @Override
    public void end(boolean inturrupted) {
        intakeSubsystem.setMotorPower(0);
    }
}
