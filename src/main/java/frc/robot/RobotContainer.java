package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.Constants.EverybotConstants;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EverybotArm;
import frc.robot.subsystems.EverybotArmMotionMagic;
import frc.robot.subsystems.EverybotClimber;
import frc.robot.subsystems.EverybotIntake;

public class RobotContainer {

    public enum Climber {
        LOW,
        TRAVERSAL;
    }

    public Drivetrain drivetrain = new Drivetrain();
    public EverybotArm everybotArm = new EverybotArm();
    public EverybotClimber everybotClimber = new EverybotClimber();
    public EverybotIntake everybotIntake = new EverybotIntake();
    public EverybotArmMotionMagic everybotArmMotionMagic = new EverybotArmMotionMagic();

    public PS4Controller ps4Controller = new PS4Controller(0);
    public PS4Controller ps4Controller2 = new PS4Controller(1);
    
    public SendableChooser<CommandGroupBase> autoChooser;
    public SendableChooser<Climber> climberChooser;

    public Climber climber;

    public RobotContainer() {
        configureButtonBindings();
        SmartDashboard.putBoolean("arm moving", false);
        initSmartDashboard();
        drivetrain.compressor.enableDigital();
    }

    private void configureButtonBindings() {
        // Assign instantcommands to each PS4 button
        // Could move to OI later

        SmartDashboard.putData("move arm to first rung", new InstantCommand(() -> {
            everybotClimber.moveClimber(EverybotConstants.kTicksToLowRung);
            SmartDashboard.putBoolean("arm moving", true);
        }));

        SmartDashboard.putData("climb part 2",
            new InstantCommand(() -> {
                everybotClimber.moveClimber(EverybotConstants.kTicksToClimbLowRung);
            })
        );
    }

    public void initSmartDashboard() {
        autoChooser = new SendableChooser<CommandGroupBase>();

        autoChooser.setDefaultOption("leave tarmac :)", 
            new SequentialCommandGroup(
                // drive for 1 second with power 0.5, then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> drivetrain.setPower(0.5, 0.5))
                ), 
                new InstantCommand(() -> drivetrain.setPowerZero())
            )
        );
        
        // autoChooser.addOption("shoot ball and leave tarmac :)", 
        //     new SequentialCommandGroup(
        //         // outtake for 1 second and then set power zero
        //         new ParallelDeadlineGroup(
        //             new WaitCommand(1), 
        //             new InstantCommand(() -> everybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake))
        //         ), 
        //         new InstantCommand(() -> everybotIntake.setPowerZero()),

        //         // drive for 1 second with power 0.5, then set power zero
        //         new ParallelDeadlineGroup(
        //             new WaitCommand(1), 
        //             new InstantCommand(() -> drivetrain.setPower(0.5, 0.5))
        //         ), 
        //         new InstantCommand(() -> drivetrain.setPowerZero())
        //     )
        // );

        
        autoChooser.addOption("move forward, shoot ball, and leave tarmac ",
            new SequentialCommandGroup(
                // outtake for 1 second and then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(3), 
                    new InstantCommand(() -> drivetrain.setPower(-0.5, -0.5))
                ),

                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> everybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake))
                ), 
                new InstantCommand(() -> everybotIntake.setPowerZero()),

                // drive for 1 second with power 0.5, then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(3), 
                    new InstantCommand(() -> drivetrain.setPower(0.5, 0.5))
                ), 
                new InstantCommand(() -> drivetrain.setPowerZero())
            )
        );
        
        
        autoChooser.addOption("delay 5s then taxi",
            new SequentialCommandGroup(
                new WaitCommand(5),
                // outtake for 1 second and then set power zero
                // new ParallelDeadlineGroup(
                //     new WaitCommand(3), 
                //     new InstantCommand(() -> drivetrain.setPower(-0.4, -0.5))
                // ),

                // new ParallelDeadlineGroup(
                //     new WaitCommand(1), 
                //     new InstantCommand(() -> everybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake))
                // ), 
                // new InstantCommand(() -> everybotIntake.setPowerZero()),

                // drive for 1 second with power 0.5, then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> drivetrain.setPower(0.5, 0.5)),
                
                new InstantCommand(() -> drivetrain.setPowerZero())))
            
        );

        SmartDashboard.putData(autoChooser);

        // TODO: implement chaning operator control based on which climb is chosen.

        climberChooser = new SendableChooser<Climber>();

        climberChooser.addOption("Select Low climb", Climber.LOW);
        climberChooser.setDefaultOption("Select Traversal climb", Climber.TRAVERSAL);
        
        SmartDashboard.putData(" Reset Climber Encoders ", new InstantCommand(() -> everybotClimber.climberMaster.setSelectedSensorPosition(0)));
    }

    public void reportToSmartDashboard() {
        drivetrain.reportToSmartDashboard();

        SmartDashboard.putNumber(" Arm Position ", everybotArm.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Intake Stator Current ", everybotIntake.everybotIntake.getStatorCurrent());
        SmartDashboard.putNumber(" Intake Supply Current ", everybotIntake.everybotIntake.getSupplyCurrent());

    }
}
