package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class DriveWithDelay extends SequentialCommandGroup{
    /**
   * Creates a new DriveWithDelay.
   *
   * @param drive The drive subsystem this command will run on
   */
    public DriveWithDelay(Drivetrain drive) {

        addCommands(
            new WaitCommand(5),

            new ParallelDeadlineGroup(
                new WaitCommand(3),
                new Drive(0.5, drive)),

            new Drive(0.0, drive)
        );
    }
}
