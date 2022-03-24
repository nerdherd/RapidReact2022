package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Arm;

public class ArmDetectCurrent extends CommandBase {
    
    @Override
    public boolean isFinished() {
        if (Arm.arm.getSupplyCurrent() > Arm.kArmDesiredHoldPow + Arm.kArmHitCurrentChange) {
            return true;
        } else {
            return false;
        }
    }

}
