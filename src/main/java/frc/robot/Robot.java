// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.Turret;

public class Robot extends TimedRobot {
  public static SendableChooser<Command> autoChooser;
  public static Command m_autonomousCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() { 
    CommandScheduler.getInstance().cancelAll();
    
    Limelight limelight = new Limelight();
    Turret turret = new Turret(limelight);
    
    robotContainer = new RobotContainer();
    
    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02, robotContainer); 
  }
  
  @Override
  public void teleopInit() { 
    robotContainer.configureButtonBindings();
    robotContainer.initSubsystems();
  }

  @Override
  public void teleopPeriodic() { 

    robotContainer.reportToSmartDashboard();
    robotContainer.configurePeriodic();
    
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    CommandGroupBase command = robotContainer.autoChooser.getSelected();

    if (command != null) {
      command.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}