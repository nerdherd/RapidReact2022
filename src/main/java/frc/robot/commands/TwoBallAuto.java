/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;

import java.util.List;
import java.lang.Math;
import java.time.Instant;

// import com.nerdherd.lib.drivetrain.experimental.Drivetrain;

import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.Roller;
import frc.robot.subsystems.shooter.Turret;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto(Drivetrain drive, Flywheel flywheel, Indexer indexer, Intake intake) {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.kRamseteS, DriveConstants.kRamseteV, DriveConstants.kRamseteA),
        drive.m_kinematics, 
        DriveConstants.kRamseteMaxVolts);
        
        var autoCentripetalAccelerationConstraint = new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAcceleration);
    
        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel);
        config.addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));

        Trajectory tarmacToTerminal = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0.8116)),
            List.of(new Translation2d(0.459, 0.366)),
            new Pose2d(0.942, 0.635, new Rotation2d(0.7636)),
            config
            );


        RamseteCommand driveTarmacToTerminal = new RamseteCommand(tarmacToTerminal,
            drive::getPose2d,
            new RamseteController(1.05, 0.14),
            new SimpleMotorFeedforward(DriveConstants.kRamseteS, DriveConstants.kRamseteV, DriveConstants.kRamseteA),
            drive.m_kinematics,
            drive::getCurrentSpeeds,
            new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD),
            new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
            drive::setVoltage, drive);

        addCommands(
            new InstantCommand(() -> flywheel.setPercent(0.375, -0.3)),
            new WaitCommand(2),
            new InstantCommand(() -> indexer.setPercent(0.9, 0.45)),
            new InstantCommand(() -> flywheel.setPercentZero()),
            new InstantCommand(() -> indexer.setPercentZero()),
            new InstantCommand(() -> intake.ReadyIntake()), // might need to stow here, not ready
            new ParallelCommandGroup(new InstantCommand(() -> flywheel.setPercent(0.6, -0.4)), driveTarmacToTerminal),
            new ParallelCommandGroup(new InstantCommand(() -> indexer.setPercent(IndexerConstants.kIndexerPercent, IndexerConstants.kIndexerPercent/2)), new InstantCommand(() -> intake.StowIntake()))
        );

    }

}
