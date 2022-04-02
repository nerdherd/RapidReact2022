package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.climber.Arm;
import frc.robot.subsystems.climber.Elevator;
import frc.robot.subsystems.climber.EverybotClimber;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EverybotConstants;
import frc.robot.commands.ToggleHookPiston;
import frc.robot.commands.autos.DriveWithDelay;
import frc.robot.commands.autos.DriveWithoutDelay;

// import frc.robot.subsystems.climber.ArmMotionMagic;
public class RobotContainer {

    public enum Climber {
        LOW,
        TRAVERSAL,
        TRAVERSAL_MOTIONMAGIC;
    }

    public Drivetrain drivetrain = new Drivetrain();
    public EverybotClimber everybotClimber = new EverybotClimber();

    public PS4Controller ps4Controller;
    public PS4Controller ps4Controller2;
    
    public SendableChooser<CommandGroupBase> autoChooser;
    public SendableChooser<Climber> climberChooser;

    public Climber climber;

    public Arm arm = new Arm();
    public Elevator elevator = new Elevator();

    public JoystickButton dCircle;
    public JoystickButton dTriangle;
    public JoystickButton oCross;
    public JoystickButton oTriangle;
    public JoystickButton oCircle;
    public JoystickButton oSquare;
    public JoystickButton oL1;
    public JoystickButton oL2;
    public JoystickButton oR1;

    public RobotContainer() {
        SmartDashboard.putBoolean("arm moving", false);
        initSmartDashboard();

        ps4Controller = new PS4Controller(0);
        ps4Controller2 = new PS4Controller(1);


        dCircle = new JoystickButton(ps4Controller, Button.kCircle.value); // Actually square button
        dTriangle = new JoystickButton(ps4Controller, Button.kTriangle.value); // Actually triangle button
        
        oCross = new JoystickButton(ps4Controller2, Button.kCross.value); // Actually circle button
        oTriangle = new JoystickButton(ps4Controller2, Button.kTriangle.value); // Actually triangle button
        oCircle = new JoystickButton(ps4Controller2, Button.kCircle.value); // Actually square button
        oSquare = new JoystickButton(ps4Controller2, Button.kSquare.value); // Actually cross button
        oL1 = new JoystickButton(ps4Controller2, Button.kL1.value);
        oL2 = new JoystickButton(ps4Controller2, Button.kL2.value);
        oR1 = new JoystickButton(ps4Controller2, Button.kR1.value);
    }

    // test if this works.

    public void configureButtonBindings() {
        // Assign instantcommands to each PS4 button
        if (climberChooser.getSelected() == Climber.TRAVERSAL) {

            // ====================== ELEVATOR FUNCTIONS ====================== //   

            oSquare.whileActiveContinuous(new InstantCommand(() -> 
                elevator.moveElevatorDown()));
            
            oSquare.whenPressed(new InstantCommand(() ->
                SmartDashboard.putString("Button State ", "Operator Square")));

            oTriangle.whileActiveContinuous(new InstantCommand(() ->
                elevator.moveElevatorUp()));
            
            oTriangle.whenPressed(new InstantCommand(() ->
                SmartDashboard.putString("Button State ", "Operator Triangle")));

            oCircle.whileActiveContinuous(new InstantCommand(() ->
                elevator.moveElevatorExtend()));
            
            oTriangle.whenPressed(new InstantCommand(() ->
                SmartDashboard.putString("Button State ", "Operator Circle")));

        } 

        else if (climberChooser.getSelected() == Climber.TRAVERSAL_MOTIONMAGIC) {
             // ====================== ARM FUNCTIONS ====================== //

            oCross.whenPressed(new InstantCommand(() -> {
                arm.setPositionMotionMagic(ClimberConstants.kTicksToRungAngle);
                SmartDashboard.putString("Button State ", "Operator Cross");
                SmartDashboard.putString(" Running Command ", " Rotate Arm Pos 1");
                SmartDashboard.putNumber(" Arm Target Position", ClimberConstants.kTicksToRungAngle);
            }));

            oSquare.whenPressed(new InstantCommand(() -> {
                arm.setPositionMotionMagic(ClimberConstants.kTicksToClearRung);
                SmartDashboard.putString("Button State ", "Operator Square");
                SmartDashboard.putString(" Running Command ", " Rotate Arm Pos 2");
                SmartDashboard.putNumber(" Arm Target Position", ClimberConstants.kTicksToClearRung);
            }));

            oTriangle.whenPressed(new InstantCommand(() -> {
                arm.setPositionMotionMagic(ClimberConstants.kTicksToVertical);
                SmartDashboard.putString("Button State ", "Operator Cross");
                SmartDashboard.putString(" Running Command ", "Rotate Arm Vertical");
                SmartDashboard.putNumber(" Arm Target Position ", ClimberConstants.kTicksToVertical);
            }));

            oCircle.whenPressed(new InstantCommand(() -> {
                elevator.setPositionMotionMagic(ClimberConstants.kElevatorTicksUp);
                SmartDashboard.putString(" Button State ", "Operator Circle ");
                SmartDashboard.putString(" Running Command ", "Elevator Extend Low Pos");
                SmartDashboard.putNumber(" Elevator Target Position", ClimberConstants.kElevatorTicksUp);
            }));

            oL1.whenPressed(new InstantCommand(() -> {
                elevator.setPositionMotionMagic(ClimberConstants.kElevatorTicksExtend);
                SmartDashboard.putString(" Button State ", "Operator L1");
                SmartDashboard.putString(" Running Command ", "Elevator Extend High Pos");
                SmartDashboard.putNumber(" Elevator Target Position ", ClimberConstants.kElevatorTicksExtend);
            }));

            oR1.whenPressed(new InstantCommand(() -> {
                elevator.setPositionMotionMagic(ClimberConstants.kElevatorTicksDown);
                SmartDashboard.putString(" Button State ", "Operator L1");
                SmartDashboard.putString(" Running Command ", "Elevator Retract Origin");
                SmartDashboard.putNumber(" Elevator Target Position ", ClimberConstants.kElevatorTicksDown);
            }));
        }
        
        else if (climberChooser.getSelected() == Climber.LOW) {
            oSquare.whenPressed(new InstantCommand(() -> {
                everybotClimber.moveClimber(1 * EverybotConstants.kTicksToLowRung);
                SmartDashboard.putBoolean("Moving to low rung", true);
                SmartDashboard.putString(" Button State ", " PS1 Square ");
            }));

            // Actually Circle
            oCross.whenPressed(new InstantCommand(() -> {
                everybotClimber.moveClimber(1 * EverybotConstants.kTicksToClimbLowRung);
                SmartDashboard.putBoolean("Climbing onto low rung", true);
                SmartDashboard.putString(" Button State ", " PS1 Cross ");
            }));
        }

        // ====================== SHIFTING FUNCTIONS ====================== //

        dTriangle.debounce(ClimberConstants.kDriverDebounce, DebounceType.kBoth).whenActive(new InstantCommand(() ->{
            drivetrain.setDriveShifterReverse();
            SmartDashboard.putString(" Button State ", "Driver Triangle");
        }));

        dCircle.debounce(ClimberConstants.kDriverDebounce, DebounceType.kBoth).whenActive(new InstantCommand(() ->{
            drivetrain.setDriveShifterForward();
            SmartDashboard.putString(" Button State ", "Driver Circle");
        }));
        
        oCross.debounce(ClimberConstants.kOperatorDebounce, DebounceType.kBoth).whenActive(new InstantCommand(() -> {
            arm.toggleClimberHook();
            SmartDashboard.putString(" Button State ", "Operator Cross");
        }));

        new ToggleHookPiston(ClimberConstants.kOperatorDeadband, ps4Controller2, arm);
    }

    public void joystickControls() {
        // ====================== DRIVE FUNCTIONS ====================== //

        double leftInput = ps4Controller.getLeftY();
        double rightInput = ps4Controller.getRightY();
        double prevLeftOutput = drivetrain.driveLeftMotors[0].getMotorOutputPercent();
        double prevRightOutput = drivetrain.driveRightMotors[0].getMotorOutputPercent();

        // Low pass filter, output = (alpha * intended value) + (1-alpha) * previous value
        double leftOutput = (DriveConstants.kDriveAlpha * leftInput) + (DriveConstants.kDriveOneMinusAlpha * prevLeftOutput);
        double rightOutput = (DriveConstants.kDriveAlpha * rightInput) + (DriveConstants.kDriveOneMinusAlpha * prevRightOutput);

        drivetrain.driveLeftMotors[0].set(ControlMode.PercentOutput, leftOutput);
        drivetrain.driveRightMotors[0].set(ControlMode.PercentOutput, rightOutput);

        // ====================== CLIMBER FUNCTIONS ====================== //

        double armInput = -ps4Controller2.getRightY();
        arm.arm.set(ControlMode.PercentOutput, armInput * 0.25, DemandType.ArbitraryFeedForward, -1 * arm.FF());
    }

    
    public void initSmartDashboard() {
        autoChooser = new SendableChooser<CommandGroupBase>();

        autoChooser.setDefaultOption("leave tarmac :)", 
            new DriveWithoutDelay(drivetrain)
        );
        
        autoChooser.addOption("delay 5s then taxi",
            new DriveWithDelay(drivetrain)
        );

        SmartDashboard.putData(autoChooser);

        climberChooser = new SendableChooser<Climber>();

        climberChooser.addOption("Select Low climb", Climber.LOW);
        climberChooser.setDefaultOption("Select Traversal climb", Climber.TRAVERSAL);
        climberChooser.addOption("Select Traversal climb (Motion Magic)", Climber.TRAVERSAL_MOTIONMAGIC);
        
        SmartDashboard.putData(climberChooser);

        SmartDashboard.putData(" Reset Climber Encoders ", new InstantCommand(() -> 
            everybotClimber.climberMaster.setSelectedSensorPosition(0)));
        
        SmartDashboard.putData(" Move arm Angle ", new InstantCommand(() -> 
            arm.setPositionMotionMagic(ClimberConstants.kTicksToRungAngle)));

        SmartDashboard.putData( " Move arm Vertical ", new InstantCommand(() ->
            arm.setPositionMotionMagic(ClimberConstants.kTicksToVertical)));

        SmartDashboard.putData( "Move arm Clear Rung ", new InstantCommand(() ->
            arm.setPositionMotionMagic(ClimberConstants.kTicksToClearRung)));

        SmartDashboard.putData( "Reset Arm Encoder ", new InstantCommand(() -> 
            arm.resetClimbEncoder()));
        
        SmartDashboard.putData(" Reset Elevator Encoder ", new InstantCommand(() ->
            elevator.resetElevatorEncoder()));

        SmartDashboard.putData(" Command Scheduler Disable ", new InstantCommand(() -> 
            CommandScheduler.getInstance().disable()));

        SmartDashboard.putData(" Elevator Coast Mode ", new InstantCommand(() ->
            elevator.elevator.setNeutralMode(NeutralMode.Coast)));
        
        SmartDashboard.putData(" Elevator Brake Mode ", new InstantCommand(() ->
            elevator.elevator.setNeutralMode(NeutralMode.Brake)));
            
    }

    public void setNeutralModes() {
        for (int i = 0; i > drivetrain.driveLeftMotors.length; i++) {
            drivetrain.driveLeftMotors[i].setNeutralMode(NeutralMode.Coast);
            drivetrain.driveRightMotors[i].setNeutralMode(NeutralMode.Coast);
        }

        elevator.elevator.setNeutralMode(NeutralMode.Brake);
        arm.arm.setNeutralMode(NeutralMode.Brake);
    }

    public void resetEncoderPositions() {
        elevator.elevator.setSelectedSensorPosition(0);
        everybotClimber.climberMaster.setSelectedSensorPosition(0);
        arm.arm.setSelectedSensorPosition(0);

    }

    public void reportToSmartDashboard() {

        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Position ", arm.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Velocity ", arm.arm.getSelectedSensorVelocity());
        SmartDashboard.putNumber(" Arm Voltage ", arm.arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Arm Angle Conversion ", arm.ticksToAngle());
        SmartDashboard.putNumber(" Elevator Position ", elevator.elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Elevator Voltage ", elevator.elevator.getMotorOutputVoltage());
        SmartDashboard.putBoolean(" Triangle Button Held ", ps4Controller2.getTriangleButton());
        SmartDashboard.putNumber(" Climber Voltage ", everybotClimber.climberMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Climber Current ", everybotClimber.climberMaster.getSupplyCurrent());
        SmartDashboard.putNumber(" Left Operator Y Axis ", ps4Controller2.getLeftY());

    }
}
