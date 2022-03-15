package frc.robot;

import com.ctre.phoenix.platform.can.AutocacheState;

import org.ejml.equation.Sequence;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class OI {
    public static PS4Controller ps4Controller = new PS4Controller(0);
    public static PS4Controller ps4Controller2 = new PS4Controller(1);

    public OI() {
        
        SendableChooser<CommandGroupBase> autoChooser = new SendableChooser<CommandGroupBase>();

        autoChooser.addOption("leave tarmac :)", 
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitCommand(1), 
                new InstantCommand(() -> Drivetrain.setPower(0.5, 0.5))
            ), 
            new InstantCommand(() -> Drivetrain.setPowerZero())
            )
        );

        SmartDashboard.putData(autoChooser);
    }
}
