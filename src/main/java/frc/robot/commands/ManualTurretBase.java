package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Turret;

public class ManualTurretBase extends CommandBase {
    Turret turret;
    double ticksPer20ms;
    boolean stop = false;
    
    public ManualTurretBase(Turret turret, double ticksPer20ms) {
        this.turret = turret;
        this.ticksPer20ms = ticksPer20ms;
    }

    @Override
    public void execute() {
        turret.changeTargetPosition(ticksPer20ms);
    }

    public void stop() {
        stop = true;
    }

    @Override
    public boolean isFinished() {
        return stop;
    }
}
