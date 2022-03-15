package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.EverybotConstants;
import frc.robot.commands.Outtake;
import frc.robot.RobotContainer;

public class BasicAuto extends SequentialCommandGroup {
    public BasicAuto(RobotContainer robotContainer) {
        addCommands(
            new Outtake(EverybotConstants.kEverybotAutoOuttake, robotContainer.everybotIntake)//,
            // new ParallelRaceGroup(new OpenLoopDrive(0.5), new WaitCommand(1))
        );
    }
}
