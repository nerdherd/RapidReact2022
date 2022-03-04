// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.everybot.Everybot;
import frc.robot.everybot.EverybotHeight;
import frc.robot.logging.Log;

public class Robot extends TimedRobot {
  @Override
  public void robotInit() { 
    Drivetrain.setupDrivetrain();
    Everybot.setUpEverybot();

    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02);
  }
  
  @Override
  public void teleopInit() { 
    Drivetrain.compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() { 
    Drivetrain.driveControllerMovement();
    Drivetrain.updateSmartDashboardForDrivetrain();

    Everybot.shooterControllerMovement();

    // SmartDashboard.putData(" Reset Elevator Encoder ", EverybotHeight.resetElevatorEncoder());
    // SmartDashboard.putNumber(" Elevator Position ", Everybot.arm.arm.getSelectedSensorPosition());
  }

  @Override
  public void autonomousInit() {
    Everybot.arm.arm.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousPeriodic() {
    Drivetrain.drive(-50, -50, 1);
  }
}