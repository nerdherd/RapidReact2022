package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.constants.ClimberConstants;

public class MoveArm extends SequentialCommandGroup {
    Arm arm;
    
    @Override
    public void execute() {
        arm = new Arm(ClimberConstants.kAngleOffset, ClimberConstants.kAngleRatio, ClimberConstants.kGravityFF, ClimberConstants.kStaticFF);
        addCommands(
            new InstantCommand(() -> arm.setPositionMotionMagic(ClimberConstants.kArmGoalAngle))
            // new InstantCommand(() -> Robot.arm.setGoal(ClimberConstants.kArmGoalAngle))
        );
    }

}
