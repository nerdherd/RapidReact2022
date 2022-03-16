package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.Constants.EverybotConstants;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EverybotArm;
import frc.robot.subsystems.EverybotArmMotionMagic;
import frc.robot.subsystems.EverybotClimber;
import frc.robot.subsystems.EverybotIntake;

public class RobotContainer {
    public OI oi;
    public Drivetrain drivetrain = new Drivetrain();
    public EverybotArm everybotArm = new EverybotArm();
    public EverybotClimber everybotClimber = new EverybotClimber();
    public EverybotIntake everybotIntake = new EverybotIntake();
    public EverybotArmMotionMagic everybotArmMotionMagic = new EverybotArmMotionMagic();

    public RobotContainer() {
        oi = new OI(this);
        configureButtonBindings();
        SmartDashboard.putBoolean("arm moving", false);
    }

    private void configureButtonBindings() {
        // Assign instantcommands to each PS4 button
        // Could move to OI later

        // Bind everybot intake to L1 bumper
        new JoystickButton(OI.ps4Controller2, Button.kL1.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotIntake.intakeIn(EverybotConstants.kEverybotIntake);
            SmartDashboard.putString(" Button State ", "L1");
        }, everybotIntake));
        
        // Bind everybot outtake to R1 bumper
        new JoystickButton(OI.ps4Controller2, Button.kR1.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotIntake.intakeOut(EverybotConstants.kEverybotOuttake);
            SmartDashboard.putString(" Button State ", "L2");
        }, everybotIntake));

        // Bind climber up to L2 trigger
        new JoystickButton(OI.ps4Controller2, Button.kL2.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotClimber.climberUp();
        }, everybotClimber));

        // Bind climber down to R2 trigger
        new JoystickButton(OI.ps4Controller2, Button.kR2.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotClimber.climberDown();
        }, everybotIntake));
        
        // Bind intake in to triangle button
        new JoystickButton(OI.ps4Controller2, Button.kTriangle.value)
        .whenPressed(new InstantCommand(() -> { 
            everybotIntake.intakeIn(0);
        }, everybotIntake));

        new JoystickButton(OI.ps4Controller2, Button.kCross.value)
        .whenPressed(new InstantCommand(() -> {
            everybotClimber.moveClimber(EverybotConstants.kTicksToLowRung);
            SmartDashboard.putBoolean("arm moving", true);
        }));
        
        SmartDashboard.putData("move arm to first rung", new InstantCommand(() -> {
            everybotClimber.moveClimber(EverybotConstants.kTicksToLowRung);
            SmartDashboard.putBoolean("arm moving", true);
        }));

        new JoystickButton(OI.ps4Controller2, Button.kCircle.value)
        .whenPressed(new SequentialCommandGroup(
            new InstantCommand(() -> {
                everybotClimber.moveClimber(EverybotConstants.kTicksToMidRung);
            }),
            new InstantCommand(() -> {
                everybotClimber.moveClimber(EverybotConstants.kTicksToHome);
            })
        ));

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

    public void reportToSmartDashboard() {
        drivetrain.reportToSmartDashboard();

        SmartDashboard.putNumber(" Arm Position ", everybotArm.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Intake Stator Current ", everybotIntake.everybotIntake.getStatorCurrent());
        SmartDashboard.putNumber(" Intake Supply Current ", everybotIntake.everybotIntake.getSupplyCurrent());
    }
}
