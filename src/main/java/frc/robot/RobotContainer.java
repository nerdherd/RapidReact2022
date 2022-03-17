package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.subsystems.climber.ArmTrapezoid;
import frc.robot.Constants.ClimberConstants;
// import frc.robot.subsystems.climber.ArmMotionMagic;

public class RobotContainer {
    public OI oi;
    public ArmTrapezoid armTrapezoid = new ArmTrapezoid();
    // public ArmMotionMagic armMotionMagic = new ArmMotionMagic();

    public RobotContainer() {
        oi = new OI(this);
        configureButtonBindings();
        SmartDashboard.putBoolean("arm moving", false);
    }

    private void configureButtonBindings() {
        // Assign instantcommands to each PS4 button
        // Could move to OI later

        // Bind climber to rung angle to L1 bumper
        // new JoystickButton(OI.ps4Controller2, Button.kL1.value)
        // .whenPressed(new InstantCommand(() -> { 
        //    armMotionMagic.climberToAngle();
        //    SmartDashboard.putString(" Button State ", "L1");
        // }));

        // new JoystickButton(OI.ps4Controller2, Button.kL2.value)
        // .whenPressed(new InstantCommand(() -> {
        //     armMotionMagic.climberToVertical();
        //     SmartDashboard.putString(" Button State ", "L2");
        // }));

        new JoystickButton(OI.ps4Controller2, Button.kR1.value)
        .whenPressed(new InstantCommand(() -> {
            armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToRungAngle);
            SmartDashboard.putString(" Button State ", "R1");
        }));
        
    }

    public void smartDashboardButtons() {
        // SmartDashboard.putData(" Move ArmMM to Angle ", new InstantCommand(() -> 
        //     armMotionMagic.climberToAngle()));

        // SmartDashboard.putData(" Move ArmMM to Vertical ", new InstantCommand(() ->
        //     armMotionMagic.climberToVertical()));

        SmartDashboard.putData(" Move ArmTrapezoid Angle ", new InstantCommand(() -> 
            armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToRungAngle)));

        SmartDashboard.putData( " Move ArmTrapezoid Vertical ", new InstantCommand(() ->
            armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToVertical)));

        SmartDashboard.putData( "Reset Arm Encoder ", new InstantCommand(() -> 
            armTrapezoid.resetClimbEncoder()));
    }


    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Arm Position ", armTrapezoid.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Velocity ", armTrapezoid.arm.getSelectedSensorVelocity());
        SmartDashboard.putNumber(" Arm Voltage ", armTrapezoid.arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Arm Angle Conversion ", armTrapezoid.ticksToAngle());
    }
}