package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.Turret;

public class TurretBaseToAngle extends InstantCommand {
    Turret turret;
    Limelight limelight;

    public TurretBaseToAngle(Turret turret, Limelight limelight) {
        this.turret = turret;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        turret.turnToBaseAngle(-limelight.getXOffsetFromTarget());
    }
    
}
