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
    public Drivetrain drivetrain = new Drivetrain();
    public EverybotArm everybotArm = new EverybotArm();
    public EverybotClimber everybotClimber = new EverybotClimber();
    public EverybotIntake everybotIntake = new EverybotIntake();
    public EverybotArmMotionMagic everybotArmMotionMagic = new EverybotArmMotionMagic();

    public PS4Controller ps4Controller = new PS4Controller(0);
    public PS4Controller ps4Controller2 = new PS4Controller(1);
    
    public SendableChooser<CommandGroupBase> autoChooser;

    public RobotContainer() {
        configureButtonBindings();
        SmartDashboard.putBoolean("arm moving", false);
        initSmartDashboard();
    }

    private void configureButtonBindings() {
        // Assign instantcommands to each PS4 button
        // Could move to OI later

        // Bind everybot intake to L1 bumper
        new JoystickButton(ps4Controller2, Button.kL1.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotIntake.intakeIn(EverybotConstants.kEverybotIntake);
            SmartDashboard.putString(" Button State ", "L1");
        }, everybotIntake));
        
        // Bind everybot outtake to R1 bumper
        new JoystickButton(ps4Controller2, Button.kR1.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotIntake.intakeOut(EverybotConstants.kEverybotOuttake);
            SmartDashboard.putString(" Button State ", "L2");
        }, everybotIntake));
        
        // TODO: doublecheck that these buttons are working, and also make sure that the commands end when the robot is disabled.

        // Bind intake in to triangle button
        new JoystickButton(ps4Controller2, Button.kTriangle.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotIntake.intakeIn(0);
        }, everybotIntake));

        new JoystickButton(ps4Controller2, Button.kCross.value)
        .whenPressed(new InstantCommand(() -> {
            everybotClimber.moveClimber(EverybotConstants.kTicksToLowRung);
            SmartDashboard.putBoolean("arm moving", true);
        }));
        
        new JoystickButton(ps4Controller2, Button.kCircle.value)
        .whenPressed(new SequentialCommandGroup(
            new InstantCommand(() -> {
                everybotClimber.moveClimber(EverybotConstants.kTicksToMidRung);
            })

            // TODO: find new values to completely hook onto the mid rung after we align with mid.

            // ,new InstantCommand(() -> {
            //     everybotClimber.moveClimber(EverybotConstants.kTicksToHome);
            // })
        ));
        
        // Smartdashboard buttons for testing the climber

        SmartDashboard.putData("move arm to first rung", new InstantCommand(() -> {
            everybotClimber.moveClimber(EverybotConstants.kTicksToLowRung);
            SmartDashboard.putBoolean("arm moving", true);
        }));

        SmartDashboard.putData("climb part 2",
            new InstantCommand(() -> {
                everybotClimber.moveClimber(EverybotConstants.kTicksToMidRung);
            })
        );

        SmartDashboard.putData("home", new InstantCommand(
            () -> everybotClimber.moveClimber(EverybotConstants.kTicksToHome)
        ));

        // These commands are disabled for now, as the arm is mechanically unstable
        
        // Bind arm up command to square button
        // new JoystickButton(OI.ps4Controller2, Button.kSquare.value)
        // .whenPressed(new InstantCommand(() -> { 
        //    everybotArm.rotateArmToAngle(EverybotConstants.kHighAngle, EverybotConstants.kHighAngleThreshold);
        //    everybotArm.arm.set(ControlMode.PercentOutput, -0.16);
        // }, everybotIntake));

        // Bind arm down command to circle button
        // new JoystickButton(OI.ps4Controller2, Button.kSquare.value)
        // .whenPressed(new InstantCommand(() -> { 
        //    everybotArm.rotateArmToAngle(EverybotConstants.kLowAngle, EverybotConstants.kLowAngleThreshold);
        //    everybotArm.arm.set(ControlMode.PercentOutput, 0.16);
        // }, everybotIntake));
    }

    public void initSmartDashboard() {
        // auto selector stuff
        // TODO: Make sure that the autochooser is working. 

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
        
        autoChooser.addOption("shoot ball and leave tarmac :)", 
            new SequentialCommandGroup(
                // outtake for 1 second and then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> everybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake))
                ), 
                new InstantCommand(() -> everybotIntake.setPowerZero()),

                // drive for 1 second with power 0.5, then set power zero
                new ParallelDeadlineGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> drivetrain.setPower(0.5, 0.5))
                ), 
                new InstantCommand(() -> drivetrain.setPowerZero())
            )
        );

        SmartDashboard.putData(autoChooser);
    }

    public void reportToSmartDashboard() {
        drivetrain.reportToSmartDashboard();

        SmartDashboard.putNumber(" Arm Position ", everybotArm.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Intake Stator Current ", everybotIntake.everybotIntake.getStatorCurrent());
        SmartDashboard.putNumber(" Intake Supply Current ", everybotIntake.everybotIntake.getSupplyCurrent());
    }
}
