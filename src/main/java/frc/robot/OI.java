package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EverybotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EverybotIntake;

public class OI {
    public static PS4Controller ps4Controller = new PS4Controller(0);
    public static PS4Controller ps4Controller2 = new PS4Controller(1);

    public OI() {
        
        SendableChooser<CommandGroupBase> autoChooser = new SendableChooser<CommandGroupBase>();

        autoChooser.addOption("leave tarmac :)", 
            new SequentialCommandGroup(
                // drive for 1 second with power 0.5, then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> Drivetrain.setPower(0.5, 0.5))
                ), 
                new InstantCommand(() -> Drivetrain.setPowerZero())
            )
        );
        
        autoChooser.addOption("shoot ball and leave tarmac :)", 
            new SequentialCommandGroup(
                // outtake for 1 second and then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> EverybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake))
                ), 
                new InstantCommand(() -> EverybotIntake.setPowerZero()),

                // drive for 1 second with power 0.5, then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> Drivetrain.setPower(0.5, 0.5))
                ), 
                new InstantCommand(() -> Drivetrain.setPowerZero())
            )
        );

        SmartDashboard.putData(autoChooser);
    }

    public void configureButtonBindings() {

    }
}
