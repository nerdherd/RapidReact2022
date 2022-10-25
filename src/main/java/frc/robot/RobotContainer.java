package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EverybotClimber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.Roller;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.BadPS4;
import frc.robot.util.BadPS4.Button;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.Rumble;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TaxiShoot;
import frc.robot.commands.TwoBallAuto;

// import frc.robot.subsystems.climber.ArmMotionMagic;
public class RobotContainer {
    public Drivetrain drivetrain = new Drivetrain();
    public EverybotClimber everybotClimber = new EverybotClimber();

    public PS4Controller ps4Controller;
    public PS4Controller ps4Controller2;

    public JoystickButton dTriangle, dCross, dSquare, dCircle, dRBumper, dLBumper;
    public JoystickButton oTriangle, oCross, oSquare, oCircle, oRBumper, oLBumper;

    public SendableChooser<CommandGroupBase> autoChooser;

    public Flywheel flywheel = new Flywheel();   
    public Indexer indexer = new Indexer();  
    public Roller roller = new Roller();
    public Intake intake = new Intake();
    public Limelight limelight = new Limelight();
    public Turret turret = new Turret(limelight);    

    public Rumble rumble;

    public RobotContainer() {
        initSmartDashboard();
        initButtons();

        ps4Controller = new BadPS4(0);
        ps4Controller2 = new BadPS4(1);
        
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
        dTriangle.debounce(0.1).whenActive(new InstantCommand(() -> turret.toggleHood()));

        // Turret hood testing code
        SmartDashboard.putData("Hood to 15 degrees", new InstantCommand(() -> turret.turnToHoodAngle(15)));
        SmartDashboard.putData("Hood to 0 degrees", new InstantCommand(() -> turret.turnToHoodAngle(0)));
        SmartDashboard.putData("Reset hood encoder", new InstantCommand(() -> turret.resetHoodEncoder()));

        // leftbumberoperator = slow flywheel
        // rightbumperoperator = fast flywheel
        // triangle = stop

        oTriangle.whenPressed(new InstantCommand(() -> flywheel.setPercentZero()));
        oCircle.whenPressed(new InstantCommand(() -> indexer.toggleIndexer()));
        // oCross.whenPressed(new InstantCommand(() -> indexer.setPercent(IndexerConstants.kIndexerPercent)));
        // oSquare.whenPressed(new InstantCommand(() -> indexer.setPercentZero()));
        oSquare.whenPressed(new InstantCommand(() -> roller.toggleRoller(RollerConstants.kRollerPercent))); // Square
        oCross.whenPressed(new InstantCommand(() -> intake.toggleIntake())); // Cross

        dRBumper.whenPressed(new InstantCommand(() -> intake.LowerIntake()));
        dLBumper.whenPressed(new InstantCommand(() -> intake.RaiseIntake()));

        oRBumper.whenPressed(new InstantCommand(() -> flywheel.setPercent(FlywheelConstants.kFlywheelOuterTarmacPercent)));
        oLBumper.whenPressed(new InstantCommand(() -> flywheel.setPercent(FlywheelConstants.kFlywheelInnerTarmacPercent)));
        // dLeftOperator1.whenPressed(new InstantCommand(() -> roller.toggleRoller(RollerConstants.kRollerPercent)));
        // dRightOperator1.whenPressed(new InstantCommand(() -> roller.setPercentZero()));
    }

    public void initButtons() {
        dTriangle = new JoystickButton(ps4Controller, Button.kTriangle.value);
        dCross = new JoystickButton(ps4Controller, Button.kCross.value);
        dSquare = new JoystickButton(ps4Controller, Button.kSquare.value);
        dCircle = new JoystickButton(ps4Controller, Button.kCircle.value);

        dRBumper = new JoystickButton(ps4Controller, Button.kR1.value);
        dLBumper = new JoystickButton(ps4Controller, Button.kL1.value);

        oTriangle = new JoystickButton(ps4Controller2, Button.kTriangle.value);
        oCross = new JoystickButton(ps4Controller2, Button.kCross.value);
        oSquare = new JoystickButton(ps4Controller2, Button.kSquare.value);
        oCircle = new JoystickButton(ps4Controller2, Button.kCircle.value);

        oRBumper = new JoystickButton(ps4Controller, Button.kR1.value);
        oLBumper = new JoystickButton(ps4Controller, Button.kL1.value);
    }

    public void configurePeriodic() {
        indexer.setPercent(ps4Controller2.getRightY());
    }

    public void initSmartDashboard() {
        autoChooser = new SendableChooser<CommandGroupBase>();

        autoChooser.setDefaultOption("Shoot then Taxi", new TaxiShoot(drivetrain, flywheel, indexer));
        autoChooser.addOption("Two Ball Auto", new TwoBallAuto(drivetrain, flywheel, indexer, intake, roller));
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
        limelight.reportToSmartDashboard();
        turret.reportToSmartDashboard();

        SmartDashboard.putNumber(" Climber Position", everybotClimber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putBoolean(" Triangle Button Held ", ps4Controller2.getTriangleButton());
        SmartDashboard.putNumber(" Climber Voltage ", everybotClimber.climberMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Climber Current ", everybotClimber.climberMaster.getSupplyCurrent());
        SmartDashboard.putNumber(" Left Operator Y Axis ", ps4Controller2.getLeftY());
    }
}
