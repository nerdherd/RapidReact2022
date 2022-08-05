package frc.robot.subsystems.climber.commands.systemchecks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.EverybotConstants;
import frc.robot.subsystems.EverybotClimber;

public class EverybotClimberTest extends SequentialCommandGroup {

    // public static RobotContainer robotContainer = new RobotContainer();
    // public static EverybotClimber everybotClimber = new EverybotClimber();

    public EverybotClimberTest() {

        addCommands(
            new EverybotMotor(RobotContainer.everybotClimber)
            // new InstantCommand(() -> everybotClimber.moveClimber(1 * EverybotConstants.kTicksToLowRung)),
            // new InstantCommand(() -> everybotClimber.moveClimber(1 * EverybotConstants.kTicksToClimbLowRung)),
            // new InstantCommand(() -> everybotClimber.moveClimber(1 * EverybotConstants.kTicksToHome))
            
        );

    }

    
}
