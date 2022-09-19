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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EverybotClimber;
import frc.robot.subsystems.climber.ArmTrapezoid;
import frc.robot.subsystems.climber.Elevator;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Roller;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EverybotConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.Rumble;

// import frc.robot.subsystems.climber.ArmMotionMagic;
public class RobotContainer {
    public Drivetrain drivetrain = new Drivetrain();
    public EverybotClimber everybotClimber = new EverybotClimber();

    public PS4Controller ps4Controller;
    public PS4Controller ps4Controller2;

    public JoystickButton dTriangleOperator;
    public JoystickButton dCrossOperator; 
    public JoystickButton dSquareOperator; // Cross
    public JoystickButton dCircleOperator; // Square
    public JoystickButton dLeftOperator1;
    public JoystickButton dRightOperator1;

    
    public SendableChooser<CommandGroupBase> autoChooser;

    public ArmTrapezoid armTrapezoid = new ArmTrapezoid();
    public Elevator elevator = new Elevator();

    public Flywheel flywheel = new Flywheel();   
    public Indexer indexer = new Indexer();  
    public Roller roller = new Roller();                                          
                                                                                  
    private boolean m_climberShifter;                                             
       
    public Rumble rumble;

    public RobotContainer() {
        SmartDashboard.putBoolean("arm moving", false);
        initSmartDashboard();
        m_climberShifter = true;

        ps4Controller = new PS4Controller(0);
        ps4Controller2 = new PS4Controller(1);
        
        //drivetrain.compressor.enableDigital();
        rumble = new Rumble(() -> drivetrain.rightMaster.getSupplyCurrent(), 
            () -> drivetrain.rightMaster.getSelectedSensorVelocity(), ps4Controller, 0, 0);
        rumble.schedule();
    }

    // test if this works.

    public void configureButtonBindings() {
        dTriangleOperator = new JoystickButton(ps4Controller2, Button.kTriangle.value);
        dCrossOperator = new JoystickButton(ps4Controller2, Button.kCross.value);
        dSquareOperator = new JoystickButton(ps4Controller2, Button.kSquare.value);
        dCircleOperator = new JoystickButton(ps4Controller2, Button.kCircle.value);
        dLeftOperator1 = new JoystickButton(ps4Controller, Button.kTriangle.value);
        dRightOperator1 = new JoystickButton(ps4Controller, Button.kCross.value);

        dTriangleOperator.whenPressed(new InstantCommand(() -> flywheel.setPercent(FlywheelConstants.kFlywheelPercent)));
        dCrossOperator.whenPressed(new InstantCommand(() -> flywheel.setPercentZero()));
        dSquareOperator.whenPressed(new InstantCommand(() -> indexer.setPercent(IndexerConstants.kIndexerPercent)));
        dCircleOperator.whenPressed(new InstantCommand(() -> indexer.setPercentZero()));
        dLeftOperator1.whenPressed(new InstantCommand(() -> roller.toggleRoller(RollerConstants.kRollerPercent)));
        dRightOperator1.whenPressed(new InstantCommand(() -> roller.setPercentZero()));


        // TELEOP DRIVE
        double leftInput = ps4Controller.getLeftY();
        double rightInput = ps4Controller.getRightY();
        double prevLeftOutput = drivetrain.leftMaster.getMotorOutputPercent();
        double prevRightOutput = drivetrain.rightMaster.getMotorOutputPercent();

        // Low pass filter, output = (alpha * intended value) + (1-alpha) * previous value
        double leftOutput = (DriveConstants.kDriveAlpha * leftInput) + (DriveConstants.kDriveOneMinusAlpha * prevLeftOutput);
        double rightOutput = (DriveConstants.kDriveAlpha * rightInput) + (DriveConstants.kDriveOneMinusAlpha * prevRightOutput);

        drivetrain.rightMaster.set(ControlMode.PercentOutput, rightOutput);
        drivetrain.leftMaster.set(ControlMode.PercentOutput, leftOutput);
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
                    new InstantCommand(() -> drivetrain.setPower(0.5, 0.5))
                ),
                new InstantCommand(() -> drivetrain.setPowerZero())
            )
            
        );

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

    public void reportToSmartDashboard() {

        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Position ", armTrapezoid.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Velocity ", armTrapezoid.arm.getSelectedSensorVelocity());
        SmartDashboard.putNumber(" Arm Voltage ", armTrapezoid.arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Arm Angle Conversion ", armTrapezoid.ticksToAngle());
        SmartDashboard.putNumber(" Elevator Position ", elevator.elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Elevator Voltage ", elevator.elevator.getMotorOutputVoltage());
        // SmartDashboard.putBoolean(" Triangle Button Held ", ps4Controller2.getTriangleButton());
        // SmartDashboard.putNumber(" Right Operator Axis ", ps4Controller2.getRightY());
        // SmartDashboard.putNumber(" ArmMM Position ", armMotionMagic.arm.getSelectedSensorPosition());
        // SmartDashboard.putNumber(" ArmMM Velocity", armMotionMagic.arm.getSelectedSensorVelocity());
        // SmartDashboard.putNumber(" ArmMM Voltage ", armMotionMagic.arm.getMotorOutputVoltage());
        SmartDashboard.putBoolean(" Triangle Button Held ", ps4Controller2.getTriangleButton());
        SmartDashboard.putNumber(" Climber Voltage ", everybotClimber.climberMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Climber Current ", everybotClimber.climberMaster.getSupplyCurrent());
        SmartDashboard.putNumber(" Left Operator Y Axis ", ps4Controller2.getLeftY());

    }
}
