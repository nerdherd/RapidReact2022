package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EverybotClimber;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.Roller;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.Rumble;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TaxiShoot;

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
    public JoystickButton dRBumperDriver;
    public JoystickButton dLBumperDriver;
    public JoystickButton dRBumperOperator;
    public JoystickButton dLBumperOperator;

    public SendableChooser<CommandGroupBase> autoChooser;

    public Flywheel flywheel = new Flywheel();   
    public Indexer indexer = new Indexer();  
    public Roller roller = new Roller();
    public Intake intake = new Intake();                                          
                                                                               
    public Rumble rumble;

    public RobotContainer() {
        initSmartDashboard();

        ps4Controller = new PS4Controller(0);
        ps4Controller2 = new PS4Controller(1);
        
        //drivetrain.compressor.enableDigital();
        rumble = new Rumble(() -> drivetrain.rightMaster.getSupplyCurrent(), 
            () -> drivetrain.rightMaster.getSelectedSensorVelocity(), ps4Controller, 0);
        rumble.schedule();

        TankDrive tankDrive = new TankDrive(drivetrain, 
                                            () -> ps4Controller.getLeftY(), 
                                            () -> ps4Controller.getRightY());
        
        drivetrain.setDefaultCommand(tankDrive);
    }

    public void initShooter() {
        flywheel.init();
        indexer.init();
        roller.init();
        intake.init();
    }

    public void configureButtonBindings() {
        dTriangleOperator = new JoystickButton(ps4Controller2, Button.kTriangle.value);
        dCrossOperator = new JoystickButton(ps4Controller2, Button.kCross.value);
        dSquareOperator = new JoystickButton(ps4Controller2, Button.kSquare.value);
        dCircleOperator = new JoystickButton(ps4Controller2, Button.kCircle.value);
        dRBumperDriver = new JoystickButton(ps4Controller, Button.kR1.value); // Driver
        dLBumperDriver = new JoystickButton(ps4Controller, Button.kR2.value); // Driver
        dRBumperOperator = new JoystickButton(ps4Controller2, Button.kR1.value);
        dLBumperOperator = new JoystickButton(ps4Controller2, Button.kL1.value);

        // leftbumberoperator = slow flywheel
        // rightbumperoperator = fast flywheel
        // triangle = stop

        dTriangleOperator.whenPressed(new InstantCommand(() -> flywheel.setPercentZero()));
        dCrossOperator.whenPressed(new InstantCommand(() -> indexer.toggleIndexer()));
        // dSquareOperator.whenPressed(new InstantCommand(() -> indexer.setPercent(IndexerConstants.kIndexerPercent)));
        // dCircleOperator.whenPressed(new InstantCommand(() -> indexer.setPercentZero()));
        dCircleOperator.whenPressed(new InstantCommand(() -> roller.toggleRoller(RollerConstants.kRollerPercent))); // Square
        dSquareOperator.whenPressed(new InstantCommand(() -> intake.toggleIntake())); // Cross

        dRBumperDriver.whenPressed(new InstantCommand(() -> intake.LowerIntake()));
        dLBumperDriver.whenPressed(new InstantCommand(() -> intake.RaiseIntake()));

        dRBumperOperator.whenPressed(new InstantCommand(() -> flywheel.setPercent(FlywheelConstants.kFlywheelOuterTarmacPercent)));
        dLBumperOperator.whenPressed(new InstantCommand(() -> flywheel.setPercent(FlywheelConstants.kFlywheelInnerTarmacPercent)));
        // dLeftOperator1.whenPressed(new InstantCommand(() -> roller.toggleRoller(RollerConstants.kRollerPercent)));
        //dRightOperator1.whenPressed(new InstantCommand(() -> roller.setPercentZero()));

    }

    // test if this works.

    public void configurePeriodic() {
        indexer.setPercent(ps4Controller2.getRightY());
    }

    
    public void initSmartDashboard() {
        autoChooser = new SendableChooser<CommandGroupBase>();

        autoChooser.setDefaultOption("Shoot then Taxi", new TaxiShoot(drivetrain, flywheel, indexer));

        // autoChooser.setDefaultOption("leave tarmac :)", 
        //     new SequentialCommandGroup(
        //         // drive for 1 second with power 0.5, then set power zero
        //         new ParallelDeadlineGroup(
        //             new WaitCommand(1), 
        //             new InstantCommand(() -> drivetrain.setPower(0.5, 0.5))
        //         ), 
        //         new InstantCommand(() -> drivetrain.setPowerZero())
        //     )
        // );
        
        // autoChooser.addOption("delay 5s then taxi",
        //     new SequentialCommandGroup(
        //         new WaitCommand(5),
        //         new ParallelDeadlineGroup(
        //             new WaitCommand(1), 
        //             new InstantCommand(() -> drivetrain.setPower(0.5, 0.5))
        //         ),
        //         new InstantCommand(() -> drivetrain.setPowerZero())
        //     )
            
        // );
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
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
