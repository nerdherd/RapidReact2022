package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMotionMagic extends SubsystemBase {
    public static TalonSRX armMM;

    public ArmMotionMagic() {
        armMM = new TalonSRX(7);
    }

}
