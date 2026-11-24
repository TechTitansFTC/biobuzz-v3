package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Configurable
public class IntakeCommand extends CommandBase {
    public static double INTAKE_MOTOR_POWER = 1;
    public static double LEFT_INTAKE_PIVOT_SERVO_DOWN_POSITION = 0.6;
    public static double RIGHT_INTAKE_PIVOT_SERVO_DOWN_POSITION = 0.4;
    public static double LEFT_INTAKE_PIVOT_SERVO_UP_POSITION = 0.4;
    public static double RIGHT_INTAKE_PIVOT_SERVO_UP_POSITION = 0.6;
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);

        int count = 0;
        if (intakeSubsystem.getSensor1()) count++;
        if (intakeSubsystem.getSensor2()) count++;
        if (intakeSubsystem.getSensor3()) count++;
        if (intakeSubsystem.getSensor4()) count++;

        if (count >= 4) {
            intakeSubsystem.setLeftIntakePivotServoPosition(LEFT_INTAKE_PIVOT_SERVO_UP_POSITION);
            intakeSubsystem.setRightIntakePivotServoPosition(RIGHT_INTAKE_PIVOT_SERVO_UP_POSITION);
        } else {
            intakeSubsystem.setLeftIntakePivotServoPosition(LEFT_INTAKE_PIVOT_SERVO_DOWN_POSITION);
            intakeSubsystem.setRightIntakePivotServoPosition(RIGHT_INTAKE_PIVOT_SERVO_DOWN_POSITION);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotorPower(0);
    }
}
