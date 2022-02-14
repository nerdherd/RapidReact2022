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

    pcm = new PneumaticsControlModule(3);

    rightMaster = new TalonFX(30);
    leftMaster = new TalonFX(16);
    rightSlave = new TalonFX(31);  
    leftSlave = new TalonFX(17);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    // Inverted the right side
    rightMaster.setInverted(true);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
  
  }
  
  @Override
  public void teleopInit() { 
    compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() { 

    double leftInput = OI.xboxController.getLeftY();
    double rightInput = OI.xboxController.getRightY();

    rightMaster.set(ControlMode.PercentOutput, rightInput);
    leftMaster.set(ControlMode.PercentOutput, leftInput);

    // Pneumatics values
    boolean enabled = compressor.enabled();
    boolean pressureSwitch = compressor.getPressureSwitchValue();
    double current = compressor.getCurrent();

    // Gear shifting
    if (OI.xboxController.getAButtonPressed()) {
      // Shifts to high gear
      leftShifter.setForwards();
      rightShifter.setForwards();
      SmartDashboard.putString(" Button State ", "A");
      SmartDashboard.putBoolean(" Shifter ", leftShifter.isForwards());
    }

    if (OI.xboxController.getBButtonPressed()) {
      // Shifts to low gear
      leftShifter.setReverse();
      rightShifter.setReverse();
      SmartDashboard.putString(" Button State ", "B");
      SmartDashboard.putBoolean(" Shifter ", leftShifter.isReverse());
    }

    // Updates SmartDashboard
    SmartDashboard.putNumber(" Right Master Current ", rightMaster.getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave Current ", rightSlave.getSupplyCurrent());
    SmartDashboard.putNumber(" Left Master Current ", leftMaster.getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave Current ", leftSlave.getSupplyCurrent());

    SmartDashboard.putBoolean(" Compressor Enabled ", enabled);
    SmartDashboard.putBoolean(" Pressure Switch ", pressureSwitch);
    SmartDashboard.putNumber(" Compressor Current ", current);
  }
}