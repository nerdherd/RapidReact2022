// package frc.robot.subsystems.climber.commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.climber.Arm;
// import frc.robot.subsystems.climber.Elevator;

// public class ClimberReady extends SequentialCommandGroup{
    
//     public ClimberReady() {
//         addCommands(
//             new InstantCommand(() -> Arm.rotateArmToTicks(-21.0, 5.0)),
//             new InstantCommand(() -> Elevator.moveElevatortoPos(0.0, 5.0)),
//             new InstantCommand(() -> new ArmDetectCurrent()),
//             new InstantCommand(() -> Elevator.moveElevatortoPos(0.0, 5.0))
//         );
//     }

// }
