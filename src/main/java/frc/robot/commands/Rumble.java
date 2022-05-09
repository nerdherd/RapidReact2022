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
   */
  @Override
  public void initialize() {
    setRatio();

    SmartDashboard.putBoolean("started", true);
    SmartDashboard.putNumber("timestamp", Timer.getFPGATimestamp());

    logEntries = new HashMap<String, Integer>() {{
      put("Drivebase Current Velocity Ratio", dataLog.start("Drivebase Current Velocity Ratio", "double"));
      put("Drivebase Current", dataLog.start("Drivebase Current", "double"));
      put("Drivebase Velocity", dataLog.start("Drivebase Velocity", "double"));
    }};
  }

  /**
   * Set the velocity-current ratio.
   * Constantly called until the ratio is above zero
   */
  public void setRatio() {
    if (this.motorCurrentSupplier.getAsDouble() != 0) {
      velocityCurrentRatio = this.motorVelocitySupplier.getAsDouble() / this.motorCurrentSupplier.getAsDouble();
    } else {
      // will never rumble
      velocityCurrentRatio = 0;
    }
  }

  /**
   * Compare the velocity current ratio to the ratio at the current timestamp.
   * Add strength to rumble if the velocity current ratio is higher, until 5 seconds is reached.
   * <p>
   * Called every time the scheduler is run.
   */
  @Override
  public void execute() {
    // Get motor velocity and current
    double motorVelocity = this.motorVelocitySupplier.getAsDouble();
    double motorCurrent = this.motorCurrentSupplier.getAsDouble();
    
    SmartDashboard.putNumber("Motor velocity from inside haptic", motorVelocity);
    SmartDashboard.putNumber("Motor current from inside haptic", motorCurrent);
    
    double currentFPGATimestamp = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("timestamp", currentFPGATimestamp);
    SmartDashboard.putNumber("ratio", velocityCurrentRatio);

    // Set rumble to 0 if the ideal ratio is 0, and attempt to calculate the ideal ratio again
    if (velocityCurrentRatio == 0) {
      // TODO Possible bug: ratio might be set before the motors accelerate completely, so the ratio might be too low
      setRatio();
      setBothRumbles(0);
      // Check if motor velocity is above minimum velocity/current
    } else if (motorVelocity >= minimumVelocity && motorCurrent >= minimumCurrent) {
      // Get velocity-current ratio
      double ratio = motorVelocity / motorCurrent;
      SmartDashboard.putNumber(" Recent Ratio ", ratio);
      // Check if ratio is less than ideal ratio - deadband
      if (ratio < velocityCurrentRatio - deadband) {
        // Add time to time rammed
        timeRammed += (currentFPGATimestamp - lastFPGATimestamp);
        // Reset time at zero
        timeUnderRatio = 0;
      } else {
        // Add time to time under ratio
        timeUnderRatio += (currentFPGATimestamp - lastFPGATimestamp);
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

    // Multiply by 1 million because for some reason all values were E-5 in the log file
    long currentFPGATimestampLong = (long) currentFPGATimestamp * 1000000;

    // Log values
    dataLog.appendDouble(logEntries.get("Drivebase Current Velocity Ratio"), velocityCurrentRatio, currentFPGATimestampLong);
    dataLog.appendDouble(logEntries.get("Drivebase Current"), motorCurrent, currentFPGATimestampLong);
    dataLog.appendDouble(logEntries.get("Drivebase Velocity"), motorVelocity, currentFPGATimestampLong);
    
    // Set the current FPGA Timestamp to the previous one to prepare for the next iteration of the command
    lastFPGATimestamp = currentFPGATimestamp;
  }

  /**
   * Called once the command ends or is interrupted.
   * <p>
   * Finishes logging
   * @param interrupted     true if the command was interrupted/cancelled, false if ended normally
   */
  @Override
  public void end(boolean interrupted) {
    // Finish logging for all entries
    for (int entry : logEntries.values()) {
      dataLog.finish(entry);
    }
    // Close the log file
    dataLog.close();
  }

  /**
   * Returns true when this command ends.
   */
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
}
