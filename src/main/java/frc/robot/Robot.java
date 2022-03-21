// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Arm;
import frc.robot.subsystems.climber.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.logging.Log;

public class Robot extends TimedRobot {
  public RobotContainer robotContainer;

  @Override
  public void robotInit() { 
    Drivetrain.setupDrivetrain();

    robotContainer = new RobotContainer();
    Log.initAndLog("/home/lvuser/logs/", "Test", 0.04, robotContainer);

    robotContainer.elevator.elevator.setSelectedSensorPosition(0);
    robotContainer.elevator.elevator.setNeutralMode(NeutralMode.Brake);
    robotContainer.armTrapezoid.arm.setNeutralMode(NeutralMode.Brake);
    // robotContainer.armTrapezoid.arm.set(ControlMode.PercentOutput, 0.09);
  }

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber(" Arm ", robotContainer.armTrapezoid.arm.getSelectedSensorPosition());///4096*360));
    // SmartDashboard.putNumber(" Arm Vel ", robotContainer.armTrapezoid.arm.getSelectedSensorVelocity());
  }
  
  @Override
  public void teleopInit() { 
    Drivetrain.compressor.enableDigital();
    robotContainer.elevator.elevator.setSelectedSensorPosition(0);
  }


  @Override
  public void teleopPeriodic() { 
    // Drivetrain.updateSmartDashboardForDrivetrain();
    robotContainer.smartDashboardButtons();
    robotContainer.reportToSmartDashboard();
    robotContainer.configureButtonBindings();
    // robotContainer.armTrapezoid.arm.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, -1 * robotContainer.armTrapezoid.FF());
    Drivetrain.driveControllerMovement();

      // if (robotContainer.elevator.elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksUp) {
      //     robotContainer.elevator.elevator.set(ControlMode.PercentOutput, 0.16);
      //     SmartDashboard.putString(" Running Command ", "Elevator Up ");
      // } else if (robotContainer.elevator.elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksUp) {
      //     robotContainer.elevator.elevator.set(ControlMode.PercentOutput, 0);
      // }
      // SmartDashboard.putString( "Button State ", " Triangle ");
  }


  @Override
  public void autonomousInit() {
    //Drivetrain.drive(-50, -50, 10);
    Arm.arm.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousPeriodic() {
    // Arm.rotateArmToAngle(64, 5);
    SmartDashboard.putNumber(" Arm Position ", Arm.arm.getSelectedSensorPosition());
    
    // -21560 to reach mid 
    // -2675 to go down & latch
  }

  // @Override
  public void disabledInit() {
    robotContainer.elevator.elevator.setNeutralMode(NeutralMode.Brake);
  }
}