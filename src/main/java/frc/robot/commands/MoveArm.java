package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.ClimberConstants;

public class MoveArm extends SequentialCommandGroup {
    
    @Override
    public void execute() {
        // addCommands(
        //     new InstantCommand(() -> Robot.arm.setGoal(ClimberConstants.kArmGoalAngle))
        // );
    }

}
