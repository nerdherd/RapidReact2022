// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.core.io.DataOutputAsStream;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.datalog.DataLog;

public class Rumble extends CommandBase {
  private final DoubleSupplier motorVelocitySupplier;
  private final DoubleSupplier motorCurrentSupplier;
  private final GenericHID controller;
  private final double deadband;

  // TODO: Should be defined in constants, but done here to avoid merge conflicts
  private final double minimumVelocity = 1;
  private final double minimumCurrent = 0.2;

  private double lastFPGATimestamp;
  private double timeRammed = 0;
  private double velocityCurrentRatio;
  private double timeUnderRatio = 0;
  private double timeCancelBuffer = 0;

  private HashMap<String, Integer> logEntries;

  private DataLog dataLog;

  /** Creates a new Rumble. 
   * Run during teleopPeriodic() to make the controller rumble when ramming into an obstacle
   * 
   * @param motorVelocity     a supplier returning the motor velocity
   * @param motorCurrent      a supplier returning the motor current (supply)
   * @param controller        the generic HID controller to rumble
   * @param deadband          a deadband that dictates how far away from the current-velocity ratio the motor can stray
   * @param timeCancelBuffer  the time span before the robot is marked as safe and not ramming
   * @return                  a new Rumble command
  */
  public Rumble(DoubleSupplier motorVelocity, DoubleSupplier motorCurrent, 
                GenericHID controller, double deadband, double timeCancelBuffer) {
    this.motorVelocitySupplier = motorVelocity;
    this.motorCurrentSupplier = motorCurrent;
    this.controller = controller;
    this.deadband = deadband;
    this.timeCancelBuffer = timeCancelBuffer;

    dataLog = new DataLog("/home/lvuser/logs", "RumbleLog");
  }

  /**
   * Initialize the velocity to current ratio.
   * Called when the command is initially scheduled.
   * <p>
   * Can also be mannually called to reset the ratio
   */
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("started", true);
    if (this.motorCurrentSupplier.getAsDouble() != 0) {
      velocityCurrentRatio = this.motorVelocitySupplier.getAsDouble() / this.motorCurrentSupplier.getAsDouble();
    } else {
      // will never rumble 
      velocityCurrentRatio = 0;
    }
    SmartDashboard.putNumber("timestamp", Timer.getFPGATimestamp());

    logEntries = new HashMap<String, Integer>() {{
      put("Drivebase Current Velocity Ratio", dataLog.start("Drivebase Current Velocity Ratio", "double"));
      put("Drivebase Current", dataLog.start("Drivebase Current", "double"));
      put("Drivebase Velocity", dataLog.start("Drivebase Velocity", "double"));
    }};
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get motor velocity and current
    double motorVelocity = this.motorVelocitySupplier.getAsDouble();
    double motorCurrent = this.motorCurrentSupplier.getAsDouble();
    
    SmartDashboard.putNumber("Motor velocity from inside haptic", motorVelocity);
    SmartDashboard.putNumber("Motor current from inside haptic", motorCurrent);
    
    SmartDashboard.putNumber("timestamp", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("ratio", velocityCurrentRatio);

    lastFPGATimestamp = Timer.getFPGATimestamp();
    
    // Set rumble to 0 if the ideal ratio is 0
    if (velocityCurrentRatio == 0) {
      initialize();
      setBothRumbles(0);
    }
    
    // Check if motor velocity is above minimum velocity/current
    else if (motorVelocity >= minimumVelocity && motorCurrent >= minimumCurrent) {
      // Get velocity-current ratio
      double ratio = motorVelocity / motorCurrent;
      SmartDashboard.putNumber(" Recent Ratio ", ratio);
      // Check if ratio is less than ideal ratio - deadband
      if (ratio < velocityCurrentRatio - deadband) {
        // Add time to time rammed
        timeRammed += (Timer.getFPGATimestamp() - lastFPGATimestamp);
        // Reset time at zero
        timeUnderRatio = 0;
      } else {
        // Add time to time at zero
        timeUnderRatio += (Timer.getFPGATimestamp() - lastFPGATimestamp);
        // Only reset time rammed if time at zero is greater than buffer
        if (timeUnderRatio >= timeCancelBuffer) {
          // Reset the time rammed
          timeRammed = 0;
        }
      }
    }

    // Set the rumble strength (from 0 to 1 on an exponential scale)
    double rumbleStrength = (timeRammed / 5) * (timeRammed / 5);
    SmartDashboard.putNumber("Rammed Time", (int) timeRammed);
    setBothRumbles(rumbleStrength); 

    // dataLog.start("Drivebase Current Velocity Ratio", "double");
    // dataLog.start("Drivebase Current", "double");
    // dataLog.start("Drivebase Velocity", "double");

    dataLog.appendDouble(logEntries.get("Drivebase Current Velocity Ratio"), velocityCurrentRatio, (long)Timer.getFPGATimestamp());
    dataLog.appendDouble(logEntries.get("Drivebase Current"), motorCurrent, (long)Timer.getFPGATimestamp());
    dataLog.appendDouble(logEntries.get("Drivebase Velocity"), motorVelocity, (long)Timer.getFPGATimestamp());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Set left and right rumble to the same strength
   * 
   * @param rumbleStrength  the strength of the rumble [0, 1]
   */
  private void setBothRumbles(double rumbleStrength) {
    controller.setRumble(GenericHID.RumbleType.kLeftRumble, rumbleStrength);
    controller.setRumble(GenericHID.RumbleType.kRightRumble, rumbleStrength); 
  }

  public void DisableLog() {
    if (dataLog != null && logEntries != null) {
      for (int entry : logEntries.values()) {
        dataLog.finish(entry);
      }
  
      dataLog.close();
    }
  }
}
