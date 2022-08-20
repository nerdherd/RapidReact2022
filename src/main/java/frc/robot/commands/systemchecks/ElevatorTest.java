package frc.robot.commands.systemchecks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.commands.elevator.ElevatorDown;
import frc.robot.subsystems.climber.commands.elevator.ElevatorExtend;

public class ElevatorTest extends SequentialCommandGroup {
    
    public ElevatorTest() {
        addCommands(
            new ElevatorExtend(),
            new ElevatorDown()
        );
    }

}
