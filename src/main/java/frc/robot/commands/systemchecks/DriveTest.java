package frc.robot.commands.systemchecks;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.SystemCheckConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTest extends SequentialCommandGroup {

    /*private void driveForSecondsLeft(double leftPower, double seconds) {
        RobotContainer.drivetrain.setPowerLeft(leftPower);
        Timer.delay(seconds);
        RobotContainer.drivetrain.setPowerLeft(0);
    }

    private void driveForSecondsRight(double rightPower, double seconds) {
        RobotContainer.drivetrain.setPowerRight(rightPower);
        Timer.delay(seconds);
        RobotContainer.drivetrain.setPowerRight(0);
    }*/

    public DriveTest() {

        addCommands(
            new DriveMotor(RobotContainer.drivetrain)    

            // // Drive both sides forwards independently
            // new InstantCommand(() -> SmartDashboard.putString(" Command 1 ", " Run ")),
            // new InstantCommand(() -> SmartDashboard.putString(" Command 2 ", " Run ")),

            // // Drive both sides backwards independently
            // new InstantCommand(() -> driveForSecondsLeft(-SystemCheckConstants.kDrivePower, 1)),
            // new InstantCommand(() -> driveForSecondsRight(-SystemCheckConstants.kDrivePower, 1)),

            // // Drive both sides forwards together
            // new InstantCommand(() -> RobotContainer.drivetrain.drive(SystemCheckConstants.kDrivePower, 
            //     SystemCheckConstants.kDrivePower, 1)),
            // new InstantCommand(() -> RobotContainer.drivetrain.setPower(0, 0)),

            // // Drive both sides backwards together
            // new InstantCommand(() -> RobotContainer.drivetrain.drive(-SystemCheckConstants.kDrivePower, 
            //     SystemCheckConstants.kDrivePower, 1)),
            // new InstantCommand(() -> RobotContainer.drivetrain.setPower(0, 0))

        
        );

    }
    
}
