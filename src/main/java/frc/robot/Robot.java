// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Arm;
import frc.robot.subsystems.climber.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.logging.Log;

public class Robot extends TimedRobot {
  public RobotContainer robotContainer;

  @Override
  public void robotInit() { 
    Drivetrain.setupDrivetrain();
    Elevator.elevator.setSelectedSensorPosition(0);
    Arm.arm.setSelectedSensorPosition(0);
    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02);

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber(" Arm ", Arm.arm.getSelectedSensorPosition());///4096*360));
    SmartDashboard.putNumber(" Arm Vel ", Arm.arm.getSelectedSensorVelocity());
  }
  
  @Override
  public void teleopInit() { 
    Drivetrain.compressor.enableDigital();
  }


  @Override
  public void teleopPeriodic() { 
    // Drivetrain.driveControllerMovement();
    // Drivetrain.updateSmartDashboardForDrivetrain();
    SmartDashboard.putData("Move Elevator", new InstantCommand(() -> Elevator.moveElevatortoPos(-21560, 20)));
    SmartDashboard.putNumber(" Elevator Position ", Elevator.elevator.getSelectedSensorPosition());
  }

  @Override
  public void autonomousInit() {
    //Drivetrain.drive(-50, -50, 10);
    Arm.arm.setSelectedSensorPosition(0);
    Elevator.elevator.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousPeriodic() {
    // Arm.rotateArmToAngle(64, 5);
    SmartDashboard.putNumber(" Arm Position ", Arm.arm.getSelectedSensorPosition());
    SmartDashboard.putNumber(" Elevator Position ", Elevator.elevator.getSelectedSensorPosition());
    SmartDashboard.putData(" Reset Elevator ", new InstantCommand(() -> Elevator.elevator.setSelectedSensorPosition(0)));
    SmartDashboard.putData(" Reset Arm ", new InstantCommand(() -> Arm.arm.setSelectedSensorPosition(0)));
    SmartDashboard.putData(" Move Elevator ", new InstantCommand(() -> Elevator.moveElevatortoPos(-21560, 10)));
    // -21560 to reach mid 
    // -2675 to go down & latch
  }
}