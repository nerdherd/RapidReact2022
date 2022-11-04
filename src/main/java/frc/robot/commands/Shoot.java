package frc.robot.commands;
 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 
public class Shoot extends SequentialCommandGroup {
 
    Flywheel flywheel;
    Indexer indexer;
 
    public Shoot(Flywheel flywheel, Indexer indexer) {
 
        this.flywheel = flywheel;
        this.indexer = indexer;
 
        addCommands(
            new InstantCommand(() -> flywheel.setFlywheelVelocity(
                SmartDashboard.getNumber("Flywheel Velocity", 0))),
            new WaitCommand(2),
            new InstantCommand(() -> flywheel.setFeederPercent(
                SmartDashboard.getNumber("Feeder Percent", 0.4))),
            new InstantCommand(() -> indexer.setPercent(-0.9, 0)),
            new WaitCommand(2),
            new InstantCommand(() -> indexer.setPercent(-0.9, 0.45)),
            new WaitCommand(4),
            new InstantCommand(() -> flywheel.setPercentZero()),
            new InstantCommand(() -> indexer.setPercentZero())
        );
    }
}