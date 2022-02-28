// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;

//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.OI;
import frc.robot.Pneumatics.AirCompressor;
import frc.robot.Pneumatics.Piston;
import frc.robot.drive.Drivetrain;

public class Robot extends TimedRobot {
  
  private TalonFX rightMaster; // Channel 30 on CAN, 14 on PDP
  private TalonFX leftMaster; // Channel 16 on CAN, 0 on PDP
  private TalonFX rightSlave; // Channel 31 on CAN, 15 on PDP
  private TalonFX leftSlave; // Channel 17 on CAN, 1 on PDP
  private PneumaticsControlModule pcm;

  private AirCompressor compressor = new AirCompressor(3, PneumaticsModuleType.CTREPCM);
  private Piston leftShifter = new Piston(3, PneumaticsModuleType.CTREPCM, 2, 5);
  private Piston rightShifter = new Piston(3, PneumaticsModuleType.CTREPCM, 1, 4);

  @Override
  public void robotInit() { 
    Drivetrain.setupDrivetrain();
  
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
}