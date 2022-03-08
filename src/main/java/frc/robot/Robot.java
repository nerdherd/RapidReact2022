// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.everybot.Everybot;
import frc.robot.everybot.EverybotArm;
import frc.robot.everybot.EverybotHeight;
import frc.robot.logging.Log;


public class Robot extends TimedRobot {
  private double startTimestamp;
  // private double currentTimestamp;
  private double timeoutTimestamp = 3;
  public static TalonFX intakeArm = new TalonFX(15);

  @Override
  public void robotInit() { 
    Drivetrain.setupDrivetrain();
    Everybot.setUpEverybot();

    Log.initAndLog("/home/lvuser/logs/", "Test", 0.02);
  }
  
  @Override
  public void teleopInit() { 
    Drivetrain.compressor.enableDigital();
    EverybotArm.resetElevatorEncoder();
    System.out.println(" t init");

    if (Timer.getFPGATimestamp() - startTimestamp < timeoutTimestamp) {
      Drivetrain.drive(-0.5, -0.5, 1);
    }
    else {
      Drivetrain.setPowerZero();
    }
  }

  @Override
  public void teleopPeriodic() { 
    //Drivetrain.driveControllerMovement();
    Drivetrain.updateSmartDashboardForDrivetrain();

    Everybot.shooterControllerMovement();
    Everybot.updateSmartDashboardForEverybot();

    System.out.println("t period");

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
  }

  @Override
  public void autonomousInit() {
    EverybotArm.resetElevatorEncoder();
    startTimestamp = Timer.getFPGATimestamp();
    // Drivetrain.drive(-50, -50, 1);
    System.out.println("a init");
  }

  @Override
  public void autonomousPeriodic() {
    if (Timer.getFPGATimestamp() - startTimestamp < timeoutTimestamp) {
      Drivetrain.drive(-0.5, -0.5, 1);
    }
    else {
      Drivetrain.setPowerZero();
    }
    System.out.println("a period");
  }
}