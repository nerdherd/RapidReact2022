/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.logging;

import java.io.File;
import java.util.function.Supplier;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.everybot.EverybotArmMotionMagic;
import frc.robot.everybot.EverybotIntake;
import frc.robot.subsystems.drive.Drivetrain;

/**
 * Add your docs here.
 */
public class Log {

  private static final int kMaxNumFiles = 971;
  private static final String kDefaultPath = "/home/lvuser/logs/";
  private static final String kDefaultSimPath = "\\logs\\";

  public static BadLog log;
  private static Notifier logger;

  public static void init(String directory, String filename) {
    
    log = BadLog.init(getDesiredFile(directory, filename));
    createTopic("Time", () -> Timer.getFPGATimestamp());

    createTopic("RightMaster" + "/Voltage", () -> Drivetrain.rightMaster.getMotorOutputVoltage());
    createTopic("LeftMaster" + "/Voltage", () -> Drivetrain.leftMaster.getMotorOutputVoltage());
    createTopic("RightFollower" + "/Voltage", () -> Drivetrain.rightSlave.getMotorOutputVoltage());
    createTopic("LeftFollower" + "/Voltage", () -> Drivetrain.leftSlave.getMotorOutputVoltage());
    createTopic("IntakeVoltage" + "/Voltage", () -> EverybotIntake.everybotIntake.getMotorOutputVoltage());
    createTopic("IntakeArmAngle" + "/Angle", () -> EverybotArmMotionMagic.arm.getSelectedSensorPosition());
    log.finishInitialization();
  }

  public static void initAndLog(String directory, String filename, double period) {
    init(directory, filename);
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
  }

  public static void createTopic(String name, Supplier<Double> toLog) {
    BadLog.createTopic(name, "ul", toLog);
  }

  public static void createTopicStr(String name, Supplier<String> toLog) {
    BadLog.createTopicStr(name, "ul", toLog);
  }
}