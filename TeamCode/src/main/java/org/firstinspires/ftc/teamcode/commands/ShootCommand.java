package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

@Configurable
public class ShootCommand extends CommandBase {
    public static double SHOOTING_TIME = 0.7;
    public static double GATE_OPEN_POSITION = 0.7;
    public static double GATE_CLOSE_POSITION = 0.94;
    private final GateSubsystem gateSubsystem;
    private final ElapsedTime shootTimer;
    public ShootCommand(GateSubsystem gateSubsystem) {
        this.gateSubsystem = gateSubsystem;

        shootTimer = new ElapsedTime();

        addRequirements(gateSubsystem);
    }

    @Override
    public void initialize() {
        shootTimer.reset();
        gateSubsystem.setGatePosition(GATE_OPEN_POSITION);
    }

    @Override
    public boolean isFinished() {
        return shootTimer.seconds() >= SHOOTING_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        gateSubsystem.setGatePosition(GATE_CLOSE_POSITION);
    }
}
