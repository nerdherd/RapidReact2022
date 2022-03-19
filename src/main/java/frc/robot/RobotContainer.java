package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.subsystems.climber.ArmTrapezoid;
import frc.robot.subsystems.climber.Elevator;
import frc.robot.Constants.ClimberConstants;
// import frc.robot.subsystems.climber.ArmMotionMagic;
import frc.robot.logging.Log;

public class RobotContainer {
    public OI oi;
    public ArmTrapezoid armTrapezoid = new ArmTrapezoid();
    public Elevator elevator = new Elevator();
    // public ArmMotionMagic armMotionMagic = new ArmMotionMagic();

    public RobotContainer() {
        oi = new OI(this);
        configureButtonBindings();
        SmartDashboard.putBoolean("arm moving", false);
    }

    public void configureButtonBindings() {
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

        // new JoystickButton(OI.ps4Controller2, Button.kR1.value)
        // .whenPressed(new InstantCommand(() -> {
        //     armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToRungAngle);
        //     SmartDashboard.putString(" Button State ", "R1");
        // }));

        // new JoystickButton(OI.ps4Controller2, Button.kR2.value)
        // .whenPressed(new InstantCommand(() -> {
        //     armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToClearRung);
        //     SmartDashboard.putString(" Button State ", "R2");
        // }));

        // new JoystickButton(OI.ps4Controller2, Button.kL1.value)
        // .whenPressed(new InstantCommand(() -> {
        //     armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToVertical);
        //     SmartDashboard.putString(" Button State ", "L1");
        // }));

        if (OI.ps4Controller2.getL1ButtonPressed()) {
            armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToRungAngle);
            SmartDashboard.putString(" Button State ", "L1");
        }

        if (OI.ps4Controller2.getL2ButtonPressed()) {
            armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToClearRung);
            SmartDashboard.putString(" Button State ", "L2");
        }

        if (OI.ps4Controller2.getR1ButtonPressed()) {
            armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToVertical);
            SmartDashboard.putString(" Button State ", "R1");
        }

        double elevatorInput = -OI.ps4Controller2.getLeftY();
        double armInput = -OI.ps4Controller2.getRightY();
        
        if (elevator.elevator.getSelectedSensorPosition() <= ClimberConstants.kSoftLimitTicks) {     
            elevator.elevator.set(ControlMode.PercentOutput, elevatorInput * 0.2); 
            armTrapezoid.arm.set(ControlMode.PercentOutput, -0.02);      
        } else {
            elevator.elevator.set(ControlMode.PercentOutput, -0.08);
        }

        armTrapezoid.arm.set(ControlMode.PercentOutput, armInput * 0.06, DemandType.ArbitraryFeedForward, -1 * armTrapezoid.FF());

        // if (armTrapezoid.arm.getSelectedSensorPosition() <= ClimberConstants.kArmTicksDownSoftLimit
        //     && armTrapezoid.arm.getSelectedSensorPosition() >= ClimberConstants.kArmTicksUpSoftLimit) {
        //     armTrapezoid.arm.set(ControlMode.PercentOutput, armInput * 0.12);
        // } else if (armTrapezoid.arm.getSelectedSensorPosition() > ClimberConstants.kArmTicksDownSoftLimit) {
        //     armTrapezoid.arm.setNeutralMode(NeutralMode.Brake);
        //     armTrapezoid.arm.set(ControlMode.PercentOutput, -0.16);
        // } else if (armTrapezoid.arm.getSelectedSensorPosition() < ClimberConstants.kArmTicksUpSoftLimit) {
        //     armTrapezoid.arm.setNeutralMode(NeutralMode.Brake);
        //     armTrapezoid.arm.set(ControlMode.PercentOutput, 0.16);
            // armTrapezoid.arm.set(ControlMode.PercentOutput, 0.08);
        // }
        
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

        SmartDashboard.putData( "Move ArmTrapezoid Clear Rung ", new InstantCommand(() ->
            armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToClearRung)));

        SmartDashboard.putData( "Reset Arm Encoder ", new InstantCommand(() -> 
            armTrapezoid.resetClimbEncoder()));
        
        SmartDashboard.putData(" Reset Elevator Encoder ", new InstantCommand(() ->
            elevator.resetElevatorEncoder()));

        SmartDashboard.putData(" Command Scheduler Disable ", new InstantCommand(() -> 
            CommandScheduler.getInstance().disable()));
            
    }


    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Arm Position ", armTrapezoid.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Velocity ", armTrapezoid.arm.getSelectedSensorVelocity());
        SmartDashboard.putNumber(" Arm Voltage ", armTrapezoid.arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Arm Angle Conversion ", armTrapezoid.ticksToAngle());
        SmartDashboard.putNumber(" Elevator Position ", elevator.elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Elevator Voltage ", elevator.elevator.getMotorOutputVoltage());
        // SmartDashboard.putNumber(" ArmMM Position ", armMotionMagic.arm.getSelectedSensorPosition());
        // SmartDashboard.putNumber(" ArmMM Velocity", armMotionMagic.arm.getSelectedSensorVelocity());
        // SmartDashboard.putNumber(" ArmMM Voltage ", armMotionMagic.arm.getMotorOutputVoltage());
    }
}