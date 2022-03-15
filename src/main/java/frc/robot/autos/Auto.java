package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.EverybotConstants;
import frc.robot.everybot.EverybotIntake;
import frc.robot.subsystems.drive.Drivetrain;

public class Auto extends SequentialCommandGroup {
  public Auto() {
  
    addCommands(
      new InstantCommand(() -> Robot.m_currentTimestamp = Timer.getFPGATimestamp()),
      new InstantCommand(() -> Robot.m_timeElapsed = Robot.m_currentTimestamp - Robot.m_startTimestamp),
      new ParallelRaceGroup(new InstantCommand (() -> EverybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake)), new WaitCommand(1)),
      new ParallelRaceGroup(new InstantCommand (() ->  Drivetrain.drive(0.5, 0.5, 1)), new WaitCommand(3)),
      new InstantCommand(() -> Drivetrain.setPowerZero()),
      new InstantCommand(() -> EverybotIntake.setPowerZero())
    );
  }
}