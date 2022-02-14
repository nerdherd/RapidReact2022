package frc.robot.Pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AirCompressor extends SubsystemBase {

    static Compressor compressor;

    public AirCompressor(int port, PneumaticsModuleType moduleType) {
        compressor = new Compressor(port, moduleType);
    }

    public void close() {
        compressor.close();
    }

    public void disable() {
        compressor.disable();
    }

    public boolean enabled() {
        return compressor.enabled();
    }

    public void enableDigital() {
        compressor.enableDigital();
    }

    public void enableHybrid(double minPressure, double maxPressure) {
        compressor.enableHybrid(minPressure, maxPressure);
    }

    public double getAnalogVoltage() {
        return compressor.getAnalogVoltage();
    }

    public CompressorConfigType getConfigType() {
        return compressor.getConfigType();
    }

    public double getCurrent() {
        return compressor.getCurrent();
    }

    public double getPressure() {
        return compressor.getPressure();
    }

    public boolean getPressureSwitchValue() {
        return compressor.getPressureSwitchValue();
    }

    public void initSendable(SendableBuilder sendableBuilder) {
        compressor.initSendable(sendableBuilder);
    }   

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Air Compressor Analog Voltage", getAnalogVoltage());
        SmartDashboard.putNumber("Air Compressor Current", getCurrent());
        SmartDashboard.putNumber("Air Compressor Pressure", getPressure());
    }
}
