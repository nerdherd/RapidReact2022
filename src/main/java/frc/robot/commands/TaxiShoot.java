package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Turret;
 
public class TaxiShoot extends SequentialCommandGroup {
 
    Drivetrain drive;
    Flywheel flywheel;
    Indexer indexer;
 
    public TaxiShoot(Drivetrain drive, Flywheel flywheel, Indexer indexer, double shootVelocity, double feedPercent) {
 
        this.drive = drive;
        this.flywheel = flywheel;
        this.indexer = indexer;
 
        addCommands(
            new InstantCommand(() -> flywheel.setFlywheelVelocity(shootVelocity)),
            new WaitCommand(2),
            new InstantCommand(() -> flywheel.setFeederPercent(feedPercent)),
            new InstantCommand(() -> indexer.setPercent(-0.9, 0)),
            new WaitCommand(2),
            new InstantCommand(() -> indexer.setPercent(-0.9, 0.45)),
            new WaitCommand(4),
            new InstantCommand(() -> flywheel.setPercentZero()),
            new InstantCommand(() -> indexer.setPercentZero()),
            new InstantCommand(() -> drive.setPower(0.5, 0.5)),
            new WaitCommand(2),
            new InstantCommand(() -> drive.setPower(0.0, 0.0))
        );
    }
}