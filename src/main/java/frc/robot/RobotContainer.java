package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PassiveClimber;
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
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.ManualTurretBase;
import frc.robot.commands.Taxi;
import frc.robot.commands.TwoBallAuto;

// import frc.robot.subsystems.climber.ArmMotionMagic;
public class RobotContainer {
    public Drivetrain drivetrain = new Drivetrain();
    public PassiveClimber everybotClimber = new PassiveClimber();

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
    public PassiveClimber passiveClimber = new PassiveClimber();    

    public RobotContainer() {
        initSmartDashboard();

        ps4Controller = new BadPS4(0);
        ps4Controller2 = new BadPS4(1);
    }

    public void initSubsystems() {
        // flywheel.init();
        indexer.init();
        roller.init();
        intake.init();
        passiveClimber.init();
        // drivetrain.startTankDrive(ps4Controller::getLeftY, ps4Controller::getRightY);
        drivetrain.startRumble(ps4Controller);
    }

    public void configureButtonBindings() {
        initButtons();
        
        // dTriangle.debounce(0.1).whenActive(new InstantCommand(() -> turret.toggleHood()));

        // Turret hood testing code
        SmartDashboard.putData("Hood to 15 degrees", new InstantCommand(() -> turret.turnToHoodAngle(15)));
        SmartDashboard.putData("Hood to 0 degrees", new InstantCommand(() -> turret.turnToHoodAngle(0)));
        SmartDashboard.putData("Reset hood encoder", new InstantCommand(() -> turret.resetHoodEncoder()));

        oCircle.whenPressed(new InstantCommand(() -> intake.toggleIntake()));
        oTriangle.whenPressed(new InstantCommand(() -> roller.toggleRoller(RollerConstants.kRollerPercent)));
        oSquare.whenPressed(new InstantCommand(() -> indexer.toggleIndexer()));
        oCross.whenPressed(new InstantCommand(() -> flywheel.toggleFlywheel()));
        
        dTriangle.whenPressed(new InstantCommand(() -> drivetrain.shiftHigh()));
        dSquare.whenPressed(new InstantCommand(() -> drivetrain.shiftLow()));
        // passiveClimber.setPowerRight(ps4Controller2.getRightY());
        // oCircle.whenPressed(new InstantCommand(() -> passiveClimber.setPowerRight(0.0)));
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
        drivetrain.tankDrive(ps4Controller.getLeftY(), ps4Controller.getRightY());
        drivetrain.setNeutralCoast();
    }

    public void initSmartDashboard() {
        autoChooser = new SendableChooser<CommandGroupBase>();

        autoChooser.setDefaultOption("Shoot then Taxi", new Taxi(drivetrain, flywheel, indexer));
        // autoChooser.addOption("Two Ball Auto", new TwoBallAuto(drivetrain, turret, indexer, intake));
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
        roller.reportToSmartDashboard();
        flywheel.reportToSmartDashboard();
        intake.reportToSmartDashboard();
        passiveClimber.reportToSmartDashboard();

        SmartDashboard.putBoolean(" Triangle Button Held ", ps4Controller2.getTriangleButton());
        SmartDashboard.putNumber(" Left Operator Y Axis ", ps4Controller2.getLeftY());
        SmartDashboard.putBoolean(" Manual Climb ", false);
    }
}
