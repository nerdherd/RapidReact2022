// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Arm;
import frc.robot.subsystems.climber.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.logging.Log;

public class Robot extends TimedRobot {
  @Override
  public void robotInit() { 
    Drivetrain.setupDrivetrain();
    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber(" Arm ", Arm.arm.getSelectedSensorPosition());///4096*360));
  }
  
  @Override
  public void teleopInit() { 
    Drivetrain.compressor.enableDigital();
  }


  @Override
  public void teleopPeriodic() { 
    Drivetrain.driveControllerMovement();
    Drivetrain.updateSmartDashboardForDrivetrain();
  }

  @Override
  public void autonomousInit() {
    //Drivetrain.drive(-50, -50, 10);
    // Arm.arm.setSelectedSensorPosition(-10);
    Elevator.elevator.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousPeriodic() {
    // Arm.rotateArmToAngle(64, 5);
    Elevator.moveElevator(50, 3);
  }
}