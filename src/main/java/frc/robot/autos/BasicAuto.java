package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.constants.EverybotConstants;

import frc.robot.commands.OpenLoopDrive;
import frc.robot.commands.Outtake;

public class BasicAuto extends SequentialCommandGroup {
    public BasicAuto() {
        addCommands(
            
        );
    }
}
