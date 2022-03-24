/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.util.function.Supplier;

// import badlog.lib.BadLog;
import frc.robot.badlog.lib.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class Log {

  private static final int kMaxNumFiles = 971;
  private static final String kDefaultPath = "/home/lvuser/logs/";
  private static final String kDefaultSimPath = "\\logs\\";

  public static BadLog log;
  private static Notifier logger;

  public static void init(String directory, String filename, RobotContainer robotContainer) {
    
    log = BadLog.init(getDesiredFile(directory, filename));
    createTopic("Time", () -> Timer.getFPGATimestamp());

    createTopic(" Arm Position sensor " + "/Position", () -> robotContainer.armTrapezoid.arm.getSelectedSensorPosition());
    createTopic(" Arm Velocity sensor " + "/Velocity", () -> robotContainer.armTrapezoid.arm.getSelectedSensorVelocity());
    createTopic(" Arm Position trajectory " + "/Position", () -> robotContainer.armTrapezoid.arm.getActiveTrajectoryPosition());
    createTopic(" Arm Velocity trajectory " + "/Velocity", () -> robotContainer.armTrapezoid.arm.getActiveTrajectoryVelocity());
    createTopic(" Elevator Position sensor " + "/Position", () -> robotContainer.elevator.elevator.getSelectedSensorPosition());
    createTopic(" Elevator Velocity sensor " + "/Velocity", () -> robotContainer.elevator.elevator.getSelectedSensorVelocity());
    createTopic(" Elevator Position trajectory " + "/Position", () -> robotContainer.elevator.elevator.getActiveTrajectoryPosition());
    createTopic(" Elevator Velocity trajectory " + "/Velocity", () -> robotContainer.elevator.elevator.getActiveTrajectoryVelocity());
    createTopic(" Arm Angle " + "/Angle", () -> robotContainer.armTrapezoid.ticksToAngle());
    // createTopic(" Climber Position (sensor) " + "/Position", () -> robotContainer.armMotionMagic.arm.getSelectedSensorPosition());
    // createTopic(" Climber Position (sensor) " + "/Velocity", () -> robotContainer.armMotionMagic.arm.getSelectedSensorVelocity());
    // createTopic(" Climber Position (trajectory) " + "/Position", () -> robotContainer.armMotionMagic.arm.getActiveTrajectoryPosition());
    // createTopic(" Climber Position (trajectory) " + "/Velocity", () -> robotContainer.armMotionMagic.arm.getActiveTrajectoryVelocity());
    createTopic("RightMaster" + "/Voltage", () -> robotContainer.drivetrain.rightMaster.getMotorOutputVoltage());
    createTopic("LeftMaster" + "/Voltage", () -> robotContainer.drivetrain.leftMaster.getMotorOutputVoltage());
    createTopic("RightFollower" + "/Voltage", () -> robotContainer.drivetrain.rightSlave.getMotorOutputVoltage());
    createTopic("LeftFollower" + "/Voltage", () -> robotContainer.drivetrain.leftSlave.getMotorOutputVoltage());
    createTopic("Climber Velocity (sensor)", () -> robotContainer.everybotClimber.climberMaster.getSelectedSensorVelocity());
    createTopic("Climber Velocity (trajectory)", () -> robotContainer.everybotClimber.climberMaster.getActiveTrajectoryVelocity());
    createTopic("Climber position (sensor)", () -> robotContainer.everybotClimber.climberMaster.getSelectedSensorPosition());
    createTopic("Climber position (trajectory)", () -> robotContainer.everybotClimber.climberMaster.getActiveTrajectoryPosition());
    createTopic("Climber output voltage", () -> robotContainer.everybotClimber.climberMaster.getMotorOutputVoltage());
    
    log.finishInitialization();
  }

  public static void initAndLog(String directory, String filename, double period, RobotContainer robotContainer) {
    init(directory, filename, robotContainer);
    logger = new Notifier(() -> {
      Log.log();
    });
    logger.startPeriodic(period);
  }

  private static String getDesiredFile(String directory, String filename) {
    // Check that the directory exists
    String pathToUse;
    File dir = new File(directory);
    if (dir.getParentFile().isDirectory()) {
      if (!dir.isDirectory()) {
        dir.mkdir();
      }

      pathToUse = directory;
    } else {
      if (RobotBase.isReal()) {
        File defaultLogsFolder = new File(kDefaultPath);

        if (!(defaultLogsFolder.isDirectory())) {
          defaultLogsFolder.mkdir();
        }

        pathToUse = kDefaultPath;

      } else {
        pathToUse = System.getProperty("user.dir") + kDefaultSimPath;
      }
    }

    int fileNumber = 0;

    for (int i = 0; i < kMaxNumFiles; i++) {
      File file = (new File(pathToUse + filename + String.valueOf(i) + ".csv"));

      if (!file.exists() && !file.isDirectory()) {
        fileNumber = i;
        break;
      }
    }
    return pathToUse + filename + String.valueOf(fileNumber) + ".csv";
  }

  public static void log() {
    log.updateTopics();
    log.log();
    // log.flush();
  }

  public static void createTopic(String name, Supplier<Double> toLog) {
    BadLog.createTopic(name, "ul", toLog);
  }

  public static void createTopicStr(String name, Supplier<String> toLog) {
    BadLog.createTopicStr(name, "ul", toLog);
  }
}