// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.Constants.EverybotConstants;
import frc.robot.everybot.Everybot;
import frc.robot.everybot.EverybotArm;
import frc.robot.everybot.EverybotClimber;
import frc.robot.everybot.EverybotIntake;
import frc.robot.logging.Log;


public class Robot extends TimedRobot {
  public static double m_startTimestamp;
  // private double currentTimestamp;
  private double m_timeoutTimestamp = 5;
  private double m_intakeTimestamp = 1;
  public static double m_currentTimestamp;
  public static double m_timeElapsed;
  public static TalonFX intakeArm = new TalonFX(15);

  public static EverybotIntake everybotIntake;
  public static EverybotClimber everybotClimber;

  public static SendableChooser<Command> autoChooser;
  public static Command m_autonomousCommand;

  @Override
  public void robotInit() { 
    Drivetrain.setupDrivetrain();
    Everybot.setUpEverybot();
    //EverybotArm2.setUpEArm();

    everybotIntake = new EverybotIntake();

    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02);

    // autoChooser = new SendableChooser<Command>();
    // autoChooser.addOption("Abstract Basic Auto", new AbstractBasicAuto());
    // autoChooser.addOption("Basic Auto", new BasicAuto());
    // autoChooser.addOption("Auto", new Auto());
  }
  
  @Override
  public void teleopInit() { 
    Drivetrain.setupDrivetrain();
    // Drivetrain.compressor.enableDigital();
    EverybotArm.resetElevatorEncoder();
    EverybotClimber.setUpClimber();

    // if (Timer.getFPGATimestamp() - m_startTimestamp < m_timeoutTimestamp) {
    //   Drivetrain.drive(-0.5, -0.5, 1);
    // }
    // else {
    //   Drivetrain.setPowerZero();
    // }
  }

  @Override
  public void teleopPeriodic() { 
    Drivetrain.driveControllerMovement();
    Drivetrain.updateSmartDashboardForDrivetrain();

    Everybot.shooterControllerMovement();
    Everybot.updateSmartDashboardForEverybot();

    /*if (OI.ps4Controller2.getR1ButtonPressed()) {
      startTimestamp = Timer.getFPGATimestamp();
      SmartDashboard.putString(" Button State ", "R1");
      
      if (Timer.getFPGATimestamp() - startTimestamp < 0.25) {
        intakeArm.set(ControlMode.PercentOutput, -0.25);
      } else {
        intakeArm.set(ControlMode.PercentOutput, -0.08);
      }
    }

    if (OI.ps4Controller2.getR2ButtonPressed()) {
      startTimestamp = Timer.getFPGATimestamp();
      SmartDashboard.putString(" Button State ", "R2");
      
      intakeArm.set(ControlMode.PercentOutput, 0.05);
    }*/

    // if (OI.ps4Controller2.getL1ButtonPressed()) {
    //   EverybotClimber.climberMaster.set(ControlMode.PercentOutput, 0.02);
    // }
    // else if(OI.ps4Controller2.getL2ButtonPressed()) {
    //   EverybotClimber.climberMaster.set(ControlMode.PercentOutput, -0.02);
    // }
    // else {
    //   EverybotClimber.climberMaster.set(ControlMode.PercentOutput, 0.0);
    // }

  }

  @Override
  public void autonomousInit() {
    EverybotArm.resetElevatorEncoder();
    m_startTimestamp = Timer.getFPGATimestamp();

    // m_autonomousCommand = autoChooser.getSelected();
    // if (m_autonomousCommand != null) { 
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {
    m_currentTimestamp = Timer.getFPGATimestamp();
    m_timeElapsed = m_currentTimestamp - m_startTimestamp;

    if ((m_timeElapsed < m_timeoutTimestamp) && (m_timeElapsed < m_intakeTimestamp)) {
      EverybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake);
    }
    else if (m_timeElapsed < m_timeoutTimestamp && m_timeElapsed > m_intakeTimestamp) {
      Drivetrain.drive(0.5, 0.5, 1);
    }
    else {
      Drivetrain.setPowerZero();
      EverybotIntake.setPowerZero();
    }
    
    // new AbstractBasicAuto();
    // new BasicAuto();

    // new Auto();

    // CommandScheduler.getInstance().run();
  }
}