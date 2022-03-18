// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.EverybotConstants;


public class Robot extends TimedRobot {
  public static double m_startTimestamp;
  // private double currentTimestamp;
  private double m_timeoutTimestamp = 5;
  private double m_intakeTimestamp = 1;
  public static double m_currentTimestamp;
  public static double m_timeElapsed;

  public static SendableChooser<Command> autoChooser;
  public static Command m_autonomousCommand;

  public static RobotContainer robotContainer;

  @Override
  public void robotInit() { 
    robotContainer = new RobotContainer();

    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02, robotContainer);

    CommandScheduler.getInstance().cancelAll();
    // autoChooser = new SendableChooser<Command>();
    // autoChooser.addOption("Abstract Basic Auto", new AbstractBasicAuto());
    // autoChooser.addOption("Basic Auto", new BasicAuto());
    // autoChooser.addOption("Auto", new Auto());
  }
  
  @Override
  public void teleopInit() { 
    // Drivetrain.compressor.enableDigital();
    robotContainer.everybotArm.resetElevatorEncoder();

    // if (Timer.getFPGATimestamp() - m_startTimestamp < m_timeoutTimestamp) {
    //   Drivetrain.drive(-0.5, -0.5, 1);
    // }
    // else {
    //   Drivetrain.setPowerZero();
    // }
  }

  @Override
  public void teleopPeriodic() { 
    robotContainer.drivetrain.driveControllerMovement();
    robotContainer.reportToSmartDashboard();

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
    robotContainer.everybotArm.resetElevatorEncoder();
    m_startTimestamp = Timer.getFPGATimestamp();
    
    CommandGroupBase command = robotContainer.autoChooser.getSelected();

    if (command != null) {
      command.schedule();
    }
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
      robotContainer.everybotIntake.intakeOut(EverybotConstants.kEverybotAutoOuttake);
    }
    else if (m_timeElapsed < m_timeoutTimestamp && m_timeElapsed > m_intakeTimestamp) {
      robotContainer.drivetrain.drive(0.5, 0.5, 1);
    }
    else {
      robotContainer.drivetrain.setPowerZero();
      robotContainer.everybotIntake.setPowerZero();
    }
    
    // new AbstractBasicAuto();
    // new BasicAuto();

    // new Auto();

    // CommandScheduler.getInstance().run();
  }

  // cancel all commands on disable
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}