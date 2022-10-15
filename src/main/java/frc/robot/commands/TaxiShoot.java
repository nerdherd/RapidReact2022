package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Indexer;
 
public class TaxiShoot extends SequentialCommandGroup {
 
    Drivetrain drive;
    Flywheel flywheel;
    Indexer indexer;
 
    public TaxiShoot(Drivetrain drive, Flywheel flywheel, Indexer indexer) {
 
        this.drive = drive;
        this.flywheel = flywheel;
        this.indexer = indexer;
 
        addCommands(
            new InstantCommand(() -> flywheel.setPercent(FlywheelConstants.kFlywheelInnerTarmacPercent)),
            new InstantCommand(() -> indexer.setPercent(IndexerConstants.kIndexerPercent)),
            new WaitCommand(5),
            new InstantCommand(() -> flywheel.setPercentZero()),
            new InstantCommand(() -> indexer.setPercentZero()),
            // Adjust drive.setPower so robot reaches outside of tarmac
            new ParallelRaceGroup(new InstantCommand(() -> drive.setPower(0.2, 0.2)), new WaitCommand(3))
        );
 
    }
    
}