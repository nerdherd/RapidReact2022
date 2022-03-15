package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.EverybotConstants;
import frc.robot.everybot.EverybotIntake;
import frc.robot.Robot;
import frc.robot.commands.OpenLoopDrive;
import frc.robot.commands.Outtake;

public class BasicAuto extends SequentialCommandGroup {
    public BasicAuto() {
        addCommands(
            new Outtake(EverybotConstants.kEverybotAutoOuttake, Robot.everybotIntake)//,
            // new ParallelRaceGroup(new OpenLoopDrive(0.5), new WaitCommand(1))
        );
    }
}
