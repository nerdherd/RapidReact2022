package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Piston extends SubsystemBase {
    private DoubleSolenoid m_piston;

    public Piston(int moduleNumber, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        m_piston = new DoubleSolenoid(moduleNumber, moduleType, forwardChannel, reverseChannel);
    }

    public void setForwards() {
        m_piston.set(Value.kForward);
    }

    public void setReverse() {
        m_piston.set(Value.kReverse);
    }

    public void setOff() {
        m_piston.set(Value.kOff);
    }

    public void switchPosition() {
        if (getValue() == Value.kForward) {
        m_piston.set(Value.kReverse);
        }
        else if (getValue() == Value.kReverse) {
        m_piston.set(Value.kForward);
        }
    }

    public DoubleSolenoid.Value getValue() {
        return m_piston.get();
    }

    public boolean isForwards() {
        return Value.kForward == getValue();
    }

    public boolean isReverse() {
        return Value.kReverse == getValue();
    }
}
