// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot {
  public static SendableChooser<Command> autoChooser;
  public static Command m_autonomousCommand;

  // TODO: refractor code so that this doesn't have to be public static, and follows the intended use of robotContainer
  public static RobotContainer robotContainer;

  @Override
  public void robotInit() { 
    CommandScheduler.getInstance().cancelAll();
    
    robotContainer = new RobotContainer();
    
    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02, robotContainer);
    robotContainer.resetEncoderPositions();
    robotContainer.setNeutralModes();
    
  }
  
  @Override
  public void teleopInit() { 

  }


  @Override
  public void teleopPeriodic() { 
    
    robotContainer.reportToSmartDashboard();
    robotContainer.configureButtonBindings();
    
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


    // -21560 to reach mid 
    // -2675 to go down & latch
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}