package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;

@Configurable
public class FlowerIntakeCommand extends CommandBase {
    private final FlowerSubsystem flowerSubsystem;
    public static double FLOWER_SERVO_DOWN = 0.2;
    public static double FLOWER_SERVO_UP = 0.5;
    public FlowerIntakeCommand(FlowerSubsystem flowerSubsystem) {
        this.flowerSubsystem = flowerSubsystem;

        addRequirements(flowerSubsystem);
    }

    @Override
    public void initialize() {
        flowerSubsystem.setServoPosition(FLOWER_SERVO_UP);
    }

    @Override
    public void end(boolean interrupted) {
        flowerSubsystem.setServoPosition(FLOWER_SERVO_DOWN);
    }
}
