package frc.robot.subsystems.climber.commands.systemchecks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class DriveTest extends SequentialCommandGroup {
    public static RobotContainer robotContainer = new RobotContainer();
    public DriveTest() {
        
        addCommands(
            new InstantCommand(() -> robotContainer.drivetrain.setPowerLeft(0.1)),
            new WaitCommand(1),
            new InstantCommand(() -> robotContainer.drivetrain.setPowerLeft(0)),
            new InstantCommand(() -> robotContainer.drivetrain.setPowerRight(0.1)),
            new WaitCommand(1),
            new InstantCommand(() -> robotContainer.drivetrain.setPowerRight(0)),

            new InstantCommand(() -> robotContainer.drivetrain.setPowerLeft(-0.1)),
            new WaitCommand(1),
            new InstantCommand(() -> robotContainer.drivetrain.setPowerLeft(0)),
            new InstantCommand(() -> robotContainer.drivetrain.setPowerRight(-0.1)),
            new WaitCommand(1),
            new InstantCommand(() -> robotContainer.drivetrain.setPowerRight(0)),

            new InstantCommand(() -> robotContainer.drivetrain.setPower(0.1,0.1)),
            new WaitCommand(1),
            new InstantCommand(() -> robotContainer.drivetrain.setPower(0,0)),
            new InstantCommand(() -> robotContainer.drivetrain.setPower(-0.1,-0.1)),
            new WaitCommand(1),
            new InstantCommand(() -> robotContainer.drivetrain.setPower(0,0))


        );

    }
    
}
