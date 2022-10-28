package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Turret;

public class TurnTurretBase extends CommandBase {
    private Turret turret;
    private boolean stop = false;
    
    public TurnTurretBase(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void execute() {
        turret.turnToTargetPosition();
    }

    @Override
    public boolean isFinished() {
        return stop;
    }

    public void end() {
        stop = true;
    }
}
