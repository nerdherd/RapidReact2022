package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.Turret;

public class TurnToTarget extends CommandBase {
    Turret turret;
    Limelight limelight;
    public boolean turnHood = true;

    public TurnToTarget(Turret turret, Limelight limelight) {
        this.turret = turret;
        this.limelight = limelight;
    }

    @Override
    public void execute() {
        turret.turnToBaseAngle(turret.getCurrentBaseAngle() + limelight.getXOffsetFromTarget());
        if (turnHood) {
            turret.turnToHoodAngle(turret.getCurrentHoodAngle() + limelight.getYOffsetFromTarget());
        }
    }

    public void toggleHood() {
        this.turnHood = !this.turnHood;
    }

    public void toggleHood(boolean enabled) {
        this.turnHood = enabled;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
