package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.subsystems.climber.ArmMotionMagic;
import frc.robot.subsystems.climber.ArmTrapezoid;

public class RobotContainer {
    public OI oi;
    public ArmMotionMagic armMotionMagic = new ArmMotionMagic();
    public ArmTrapezoid armTrapezoid = new ArmTrapezoid();

    public RobotContainer() {
        oi = new OI(this);
        configureButtonBindings();
        SmartDashboard.putBoolean("arm moving", false);
    }

    private void configureButtonBindings() {
        // Assign instantcommands to each PS4 button
        // Could move to OI later

        // Bind climber to rung angle to L1 bumper
        new JoystickButton(OI.ps4Controller2, Button.kL1.value)
        .whenPressed(new InstantCommand(() -> { 
           armTrapezoid.climberToAngle();
           SmartDashboard.putString(" Button State ", "L1");
        }));

        new JoystickButton(OI.ps4Controller2, Button.kL2.value)
        .whenPressed(new InstantCommand(() -> {
            armTrapezoid.climberToVertical();
            SmartDashboard.putString(" Button State ", "L2");
        }));

        new JoystickButton(OI.ps4Controller2, Button.kR1.value)
        .whenPressed(new InstantCommand(() -> {
            armMotionMagic.setPositionMotionMagic(0);
            SmartDashboard.putString(" Button State ", "R1");
        }));
        
    }

    public void reportToSmartDashboard() {
    }
}