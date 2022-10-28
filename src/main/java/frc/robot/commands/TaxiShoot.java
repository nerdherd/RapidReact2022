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
    Turret turret;
    Indexer indexer;
 
    public TaxiShoot(Drivetrain drive, Turret turret, Indexer indexer) {
 
        this.drive = drive;
        this.turret = turret;
        this.indexer = indexer;
 
        addCommands(
            new InstantCommand(() -> turret.setPercent(TurretConstants.kFlywheelInnerTarmacPercent)),
            new InstantCommand(() -> indexer.setPercent(IndexerConstants.kIndexerPercent)),
            new WaitCommand(5),
            new InstantCommand(() -> turret.setPercentZero()),
            new InstantCommand(() -> indexer.setPercentZero()),
            // Adjust drive.setPower so robot reaches outside of tarmac
            new ParallelRaceGroup(new InstantCommand(() -> drive.setPower(-0.5, -0.5)), new WaitCommand(3))
        );
 
    }
    
}