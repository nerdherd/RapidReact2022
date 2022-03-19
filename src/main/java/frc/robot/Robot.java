// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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
    robotContainer = new RobotContainer();

    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02, robotContainer);

    CommandScheduler.getInstance().cancelAll();
  }
  
  @Override
  public void teleopInit() { 
    robotContainer.everybotArm.resetElevatorEncoder();
  }

  @Override
  public void teleopPeriodic() { 
    robotContainer.drivetrain.driveControllerMovement();
    robotContainer.reportToSmartDashboard();
    robotContainer.drivetrain.rightMaster.setNeutralMode(NeutralMode.Coast);
    robotContainer.drivetrain.rightSlave.setNeutralMode(NeutralMode.Coast);
    robotContainer.drivetrain.leftMaster.setNeutralMode(NeutralMode.Coast);
    robotContainer.drivetrain.leftSlave.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void autonomousInit() {
    robotContainer.everybotArm.resetElevatorEncoder();
    
    CommandGroupBase command = robotContainer.autoChooser.getSelected();

    if (command != null) {
      command.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  // cancel all commands on disable
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}