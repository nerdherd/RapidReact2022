package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Outtake;

import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.commands.Outtake;
import frc.robot.commands.OpenLoopDrive;

public class BasicAuto extends SequentialCommandGroup {
    public BasicAuto() {
        addCommands(
            // new Outtake()
        );
    }
}
