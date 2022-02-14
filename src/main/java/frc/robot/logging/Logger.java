package frc.robot.logging;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class Logger {
    
    public static final String kDate = "2022_14_14_";
    private static String m_filePath1 = "/media/sda1/logs/";
    private static String m_filePath2 = "/home/lvuser/logs/";
    private static File m_file;
    public static FileWriter m_writer;
    private static boolean writeException = false;
    private static double m_logStartTime = 0;
    private static String m_fileName;

    public static void startLog() {
		writeException = false;
		// Check to see if flash drive is mounted.
		File logFolder1 = new File(m_filePath1);
		File logFolder2 = new File(m_filePath2);
		Path filePrefix = Paths.get("");
		if (logFolder1.exists() && logFolder1.isDirectory())
			filePrefix = Paths.get(logFolder1.toString(), m_fileName);
		else
			writeException = true;

		if (!writeException) {
			int counter = 0;
			while (counter <= 99) {
				m_file = new File(filePrefix.toString() + String.format("%02d", counter) + ".csv");
				if (m_file.exists()) {
					counter++;
				} else {
					break;
				}
				if (counter == 99) {
					System.out.println("file creation counter at 99!");
				}
			}
			try {
				m_writer = new FileWriter(m_file);
				m_writer.append(
						"Time, RightPosition, LeftPosition, RightVelocity, LeftVelocity, RightVoltage, LeftVoltage, RightMasterCurrent, LeftMasterCurrent\n");
				m_writer.flush();
				m_logStartTime = Timer.getFPGATimestamp();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
	}

	public static void stopLog() {
		try {
			if (null != m_writer)
				m_writer.close();
		} catch (IOException e) {
			e.printStackTrace();
			writeException = true;
		}
	}

    public static void logDriveToCSV(TalonFX leftMaster, TalonFX rightMaster, TalonFX leftFollower, TalonFX rightFollower) {
        if (!writeException) {
			try {
				double timestamp = Timer.getFPGATimestamp() - m_logStartTime;
				m_writer.append(
                          String.valueOf(timestamp) + "," 
                        + String.valueOf(rightMaster.getSelectedSensorPosition()) + ","
						+ String.valueOf(leftMaster.getSelectedSensorPosition()) + ","
						+ String.valueOf(rightMaster.getSelectedSensorVelocity()) + ","
						+ String.valueOf(leftMaster.getSelectedSensorVelocity()) + ","
						+ String.valueOf(rightMaster.getBusVoltage()) + ","
						+ String.valueOf(leftMaster.getBusVoltage()) + ","
						+ String.valueOf(rightMaster.getSupplyCurrent()) + ","
						+ String.valueOf(leftMaster.getSupplyCurrent()) + "\n");
				m_writer.flush();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
    }

}
