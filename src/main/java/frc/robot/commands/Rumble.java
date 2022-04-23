// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rumble extends CommandBase {
  private final DoubleSupplier motorVelocitySupplier;
  private final DoubleSupplier motorCurrentSupplier;
  private final GenericHID controller;
  private final double deadband;

  // TODO: Should be defined in constants, but done here to avoid merge conflicts
  private final double minimumVelocity = 100;
  private final double minimumCurrent = 0.2;

  private double lastFPGATimestamp;
  private double timeRammed = 0;
  private double velocityCurrentRatio;

  /** Creates a new Rumble. 
   * Run during teleopPeriodic() to make the controller rumble when ramming into an obstacle
   * 
   * @param motorVelocity a supplier returning the motor velocity
   * @param motorCurrent  a supplier returning the motor current (supply)
   * @param controller    the generic HID controller to rumble
   * @param deadband      a deadband that dictates how far away from the current-velocity ratio the motor can stray
   * @return              a new Rumble command
  */
  public Rumble(DoubleSupplier motorVelocity, DoubleSupplier motorCurrent, 
                GenericHID controller, double deadband) {
    this.motorVelocitySupplier = motorVelocity;
    this.motorCurrentSupplier = motorCurrent;
    this.controller = controller;
    this.deadband = deadband;
  }

  /**
   * Initialize the velocity to current ratio.
   * Called when the command is initially scheduled.
   * <p>
   * Can also be mannually called to reset the ratio
   */
  @Override
  public void initialize() {
    if (this.motorCurrentSupplier.getAsDouble() != 0) {
      velocityCurrentRatio = this.motorVelocitySupplier.getAsDouble() / this.motorCurrentSupplier.getAsDouble();
    } else {
      // will never rumble 
      velocityCurrentRatio = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set rumble to 0 if the ideal ratio is 0
    if (velocityCurrentRatio == 0) { 
      setBothRumbles(0);
      return; 
    }

    // Get motor velocity and current
    double motorVelocity = this.motorVelocitySupplier.getAsDouble();
    double motorCurrent = this.motorCurrentSupplier.getAsDouble();

    // Check if motor velocity is above minimum velocity/current
    if (motorVelocity >= minimumVelocity && motorCurrent >= minimumCurrent) {
      // Get velocity-current ratio
      double ratio = motorVelocity / motorCurrent;
      // Check if ratio is less than ideal ratio - deadband
      if (ratio < velocityCurrentRatio - deadband) {
        // Add time to time rammed
        timeRammed += (Timer.getFPGATimestamp() - lastFPGATimestamp);
      } else {
        // TODO: Add a time period before resetting time rammed, in case there are spikes in the ratio

        // Reset the time rammed
        timeRammed = 0;
      }
    }

    lastFPGATimestamp = Timer.getFPGATimestamp();

    // Set the rumble strength (from 0 to 1 on an exponential scale)
    double rumbleStrength = (timeRammed / 5) * (timeRammed / 5);
    SmartDashboard.putNumber("Rammed Time", (int) timeRammed);
    setBothRumbles(rumbleStrength); 
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
}
