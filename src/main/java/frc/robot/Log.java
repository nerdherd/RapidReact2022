/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import badlog.lib.BadLog;
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

    createTopic("Flywheel" + "/Velocity", () -> robotContainer.flywheel.flywheel.getSelectedSensorVelocity());
    createTopic("Flywheel" + "/Position", () -> robotContainer.flywheel.flywheel.getSelectedSensorPosition());
    createTopic("Flywheel" + "/Current", () -> robotContainer.flywheel.flywheel.getStatorCurrent());
    
    createTopic("Feeder" + "/Velocity", () -> robotContainer.flywheel.feeder.getSelectedSensorVelocity());
    createTopic("Feeder" + "/Position", () -> robotContainer.flywheel.feeder.getSelectedSensorPosition());
    createTopic("Feeder" + "/Current", () -> robotContainer.flywheel.feeder.getStatorCurrent());

    createTopic("LeftMaster" + "/Voltage", () -> robotContainer.drivetrain.leftMaster.getMotorOutputVoltage());
    createTopic("RightMaster" + "/Voltage", () -> robotContainer.drivetrain.leftMaster.getMotorOutputVoltage());
    createTopic("RightFollower" + "/Voltage", () -> robotContainer.drivetrain.rightSlave.getMotorOutputVoltage());
    createTopic("LeftFollower" + "/Voltage", () -> robotContainer.drivetrain.leftSlave.getMotorOutputVoltage());

    createTopic("LeftMaster" + "/Current", () -> robotContainer.drivetrain.leftMaster.getMotorOutputVoltage());
    createTopic("RightMaster" + "/Current", () -> robotContainer.drivetrain.leftMaster.getMotorOutputVoltage());
    createTopic("RightFollower" + "/Current", () -> robotContainer.drivetrain.rightSlave.getMotorOutputVoltage());
    createTopic("LeftFollower" + "/Current", () -> robotContainer.drivetrain.leftSlave.getMotorOutputVoltage());

    createTopic("LeftMaster" + "/Position", () -> robotContainer.drivetrain.leftMaster.getSelectedSensorPosition());
    createTopic("RightMaster" + "/Position", () -> robotContainer.drivetrain.leftMaster.getSelectedSensorPosition());
    createTopic("RightFollower" + "/Position", () -> robotContainer.drivetrain.rightSlave.getSelectedSensorPosition());
    createTopic("LeftFollower" + "/Position", () -> robotContainer.drivetrain.leftSlave.getSelectedSensorPosition());

    createTopic("Indexer Top" + "/Current", () -> robotContainer.indexer.IndexerTop.getStatorCurrent());
    createTopic("Indexer Bottom" + "/Current", () -> robotContainer.indexer.IndexerBottom.getStatorCurrent());
    createTopic("Indexer Top" + "/Velocity", () -> robotContainer.indexer.IndexerTop.getSelectedSensorVelocity());
    createTopic("Indexer Bottom" + "/Velocity", () -> robotContainer.indexer.IndexerBottom.getSelectedSensorVelocity());
    createTopic("Indexer Top" + "/Position", () -> robotContainer.indexer.IndexerTop.getSelectedSensorPosition());
    createTopic("Indexer Bottom" + "/Position", () -> robotContainer.indexer.IndexerBottom.getSelectedSensorPosition());
    
    createTopic("Climber Right" + "/Current", robotContainer.passiveClimber.climberRight::getStatorCurrent);
    createTopic("Climber Right" + "/Velocity", robotContainer.passiveClimber.climberRight::getSelectedSensorVelocity);
    createTopic("Climber Right" + "/Position", robotContainer.passiveClimber.climberRight::getSelectedSensorPosition);

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

    // int fileNumber = 0;

    // for (int i = 0; i < kMaxNumFiles; i++) {
    //   File file = (new File(pathToUse + filename + String.valueOf(i) + ".csv"));

    //   if (!file.exists() && !file.isDirectory()) {
    //     fileNumber = i;
    //     break;
    //   }
    // }
    // return pathToUse + filename + String.valueOf(fileNumber) + ".csv";
    return pathToUse + filename + ".csv";
  }

  public static void log() {
    log.updateTopics();
    log.log();
  }

  public static void createTopic(String name, Supplier<Double> toLog) {
    BadLog.createTopic(name, "ul", toLog);
  }

  public static void createTopicStr(String name, Supplier<String> toLog) {
    BadLog.createTopicStr(name, "ul", toLog);
  }
}