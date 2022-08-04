package frc.robot.subsystems.climber.commands.systemchecks;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.SystemCheckConstants;

public class DriveTest extends SequentialCommandGroup {
    public static RobotContainer robotContainer = new RobotContainer();

    private void driveForSecondsLeft(double leftPower, double seconds) {
        robotContainer.drivetrain.setPowerLeft(leftPower);
        Timer.delay(seconds);
        robotContainer.drivetrain.setPowerLeft(0);
    }

    private void driveForSecondsRight(double rightPower, double seconds) {
        robotContainer.drivetrain.setPowerRight(rightPower);
        Timer.delay(seconds);
        robotContainer.drivetrain.setPowerRight(0);
    }

    public DriveTest() {
        
        addCommands(
            // Drive both sides forwards together
            new InstantCommand(() -> driveForSecondsLeft(SystemCheckConstants.kDrivePower, 1)),
            new InstantCommand(() -> driveForSecondsRight(SystemCheckConstants.kDrivePower, 1)),

            // Drive both sides backwards together
            new InstantCommand(() -> driveForSecondsLeft(-SystemCheckConstants.kDrivePower, 1)),
            new InstantCommand(() -> driveForSecondsRight(-SystemCheckConstants.kDrivePower, 1)),

            // Drive both sides forwards together
            new InstantCommand(() -> robotContainer.drivetrain.drive(SystemCheckConstants.kDrivePower, 
                SystemCheckConstants.kDrivePower, 1)),
            new InstantCommand(() -> robotContainer.drivetrain.setPower(0, 0)),

            // Drive both sides backwards together
            new InstantCommand(() -> robotContainer.drivetrain.drive(SystemCheckConstants.kDrivePower, 
                SystemCheckConstants.kDrivePower, 1)),
            new InstantCommand(() -> robotContainer.drivetrain.setPower(0, 0))
        );

    }
    
}
