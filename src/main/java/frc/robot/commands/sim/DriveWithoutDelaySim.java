package frc.robot.commands.sim;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveSim;
import frc.robot.subsystems.Drivetrain;

public class DriveWithoutDelaySim extends SequentialCommandGroup{
    /**
   * Creates a new DriveWithoutDelaySim.
   *
   * @param drive The drive subsystem this command will run on
   */
    public DriveWithoutDelaySim(Drivetrain drive) {

        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(3),
                new DriveSim(0.5, drive)),
            new DriveSim(0.0, drive)
        );
    }
}

