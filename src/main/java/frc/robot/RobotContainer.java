package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EverybotClimber;
import frc.robot.subsystems.climber.ArmTrapezoid;
import frc.robot.subsystems.climber.Elevator;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EverybotConstants;

// import frc.robot.subsystems.climber.ArmMotionMagic;
public class RobotContainer {

    public enum Climber {
        LOW,
        TRAVERSAL;
    }

    public Drivetrain drivetrain = new Drivetrain();
    public EverybotClimber everybotClimber = new EverybotClimber();

    public OI OI;
    
    public SendableChooser<CommandGroupBase> autoChooser;
    public SendableChooser<Climber> climberChooser;

    public Climber climber;

    public ArmTrapezoid armTrapezoid = new ArmTrapezoid();
    public Elevator elevator = new Elevator();

    public RobotContainer() {
        SmartDashboard.putBoolean("arm moving", false);
        initSmartDashboard();
        drivetrain.compressor.enableDigital();
    }

    // test if this works.

    public void configureButtonBindings() {
        // Assign instantcommands to each PS4 button
        if (climberChooser.getSelected() == Climber.TRAVERSAL) {
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
    
            if (OI.ps4Controller2.getR2ButtonPressed()) {
                elevator.elevator.setNeutralMode(NeutralMode.Brake);
                SmartDashboard.putString(" Button State ", "R2 ");
            }
    
            if (OI.ps4Controller2.getSquareButton()) {
                if (elevator.elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksDown ){
                    elevator.elevator.set(ControlMode.PercentOutput, -0.4);
                    SmartDashboard.putString(" Running Command ", "Elevator Down ");
                } else if (elevator.elevator.getSelectedSensorPosition() <= ClimberConstants.kElevatorTicksDown) {
                    elevator.elevator.set(ControlMode.PercentOutput, 0);
                }
                SmartDashboard.putString( "Button State ", "Square ");
            }
            
            if (OI.ps4Controller2.getTriangleButton()) {
                if (elevator.elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksUp) {
                    elevator.elevator.set(ControlMode.PercentOutput, 0.32);
                    SmartDashboard.putString(" Running Command ", "Elevator Up ");
                } else if (elevator.elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksUp) {
                    elevator.elevator.set(ControlMode.PercentOutput, 0);
                }
                SmartDashboard.putString( "Button State ", " Triangle ");
            }

            if (OI.ps4Controller2.getCircleButton()) {
                if (elevator.elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksExtend) {
                    elevator.elevator.set(ControlMode.PercentOutput, 0.32);
                    SmartDashboard.putString(" Running Command ", "Elevator Up Extend ");
                } else if (elevator.elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksExtend) {
                    elevator.elevator.set(ControlMode.PercentOutput, 0);
                }
                SmartDashboard.putString(" Button State ", " Circle ");
            }
    
            double armInput = -OI.ps4Controller2.getRightY();
            armTrapezoid.arm.set(ControlMode.PercentOutput, armInput * 0.25, DemandType.ArbitraryFeedForward, -1 * armTrapezoid.FF());
        } 
        
        else if (climberChooser.getSelected() == Climber.LOW) {
            if (OI.ps4Controller2.getCircleButtonPressed()) {
                everybotClimber.moveClimber(EverybotConstants.kTicksToLowRung);
                SmartDashboard.putBoolean("Moving to low rung", true);
            }
    
            if (OI.ps4Controller2.getTriangleButtonPressed()) {
                everybotClimber.moveClimber(EverybotConstants.kTicksToClimbLowRung);
                SmartDashboard.putBoolean("Climbing onto low rung", true);
            }
        }
                
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
        
        autoChooser.addOption("delay 5s then taxi",
            new SequentialCommandGroup(
                new WaitCommand(5),
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

        SmartDashboard.putData(" Elevator Coast Mode ", new InstantCommand(() ->
            elevator.elevator.setNeutralMode(NeutralMode.Coast)));
        
        SmartDashboard.putData(" Elevator Brake Mode ", new InstantCommand(() ->
            elevator.elevator.setNeutralMode(NeutralMode.Brake)));
            
    }

    // public void smartDashboardButtons() {
    // // SmartDashboard.putData(" Move ArmMM to Angle ", new InstantCommand(() -> 
    //     //     armMotionMagic.climberToAngle()));

    //     // SmartDashboard.putData(" Move ArmMM to Vertical ", new InstantCommand(() ->
    //     //     armMotionMagic.climberToVertical()));

    //     SmartDashboard.putData(" Move ArmTrapezoid Angle ", new InstantCommand(() -> 
    //         armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToRungAngle)));

    //     SmartDashboard.putData( " Move ArmTrapezoid Vertical ", new InstantCommand(() ->
    //         armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToVertical)));

    //     SmartDashboard.putData( "Move ArmTrapezoid Clear Rung ", new InstantCommand(() ->
    //         armTrapezoid.setPositionMotionMagic(ClimberConstants.kTicksToClearRung)));

    //     SmartDashboard.putData( "Reset Arm Encoder ", new InstantCommand(() -> 
    //         armTrapezoid.resetClimbEncoder()));
        
    //     SmartDashboard.putData(" Reset Elevator Encoder ", new InstantCommand(() ->
    //         elevator.resetElevatorEncoder()));

    //     SmartDashboard.putData(" Command Scheduler Disable ", new InstantCommand(() -> 
    //         CommandScheduler.getInstance().disable()));

    //     SmartDashboard.putData(" Elevator Coast Mode ", new InstantCommand(() ->
    //         elevator.elevator.setNeutralMode(NeutralMode.Coast)));
        
    //     SmartDashboard.putData(" Elevator Brake Mode ", new InstantCommand(() ->
    //         elevator.elevator.setNeutralMode(NeutralMode.Brake)));
            
    // }


    public void reportToSmartDashboard() {
        drivetrain.reportToSmartDashboard();

        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Position ", armTrapezoid.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Velocity ", armTrapezoid.arm.getSelectedSensorVelocity());
        SmartDashboard.putNumber(" Arm Voltage ", armTrapezoid.arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Arm Angle Conversion ", armTrapezoid.ticksToAngle());
        SmartDashboard.putNumber(" Elevator Position ", elevator.elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Elevator Voltage ", elevator.elevator.getMotorOutputVoltage());
        SmartDashboard.putBoolean(" Triangle Button Held ", OI.ps4Controller2.getTriangleButton());
        SmartDashboard.putNumber(" Right Operator Axis ", OI.ps4Controller2.getRightY());
        // SmartDashboard.putNumber(" ArmMM Position ", armMotionMagic.arm.getSelectedSensorPosition());
        // SmartDashboard.putNumber(" ArmMM Velocity", armMotionMagic.arm.getSelectedSensorVelocity());
        // SmartDashboard.putNumber(" ArmMM Voltage ", armMotionMagic.arm.getMotorOutputVoltage());
        SmartDashboard.putBoolean(" Triangle Button Held ", OI.ps4Controller2.getTriangleButton());
    }
}
