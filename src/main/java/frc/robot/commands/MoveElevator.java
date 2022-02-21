package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.constants.ClimberConstants;

public class MoveElevator extends SequentialCommandGroup {
    Elevator elevator;
    
    @Override
    public void execute() {
        elevator = new Elevator();
        addCommands(
            new ParallelRaceGroup(new WaitCommand(0), new InstantCommand(() -> elevator.moveToPosition(0)))
        );        
    }

}
