package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ArmTrapezoid extends SubsystemBase {
    public TalonSRX arm;

    public ArmTrapezoid() {
        arm = new TalonSRX(15);

        // negative goes up, positive goes down when the climber is homed at hardstop

        // config tuning params in slot 0
        arm.config_kP(0, ClimberConstants.kArmkP);
        arm.config_kI(0, ClimberConstants.kArmkI);
        arm.config_kD(0, ClimberConstants.kArmkD);
        arm.config_kF(0, ClimberConstants.kArmkF);

        arm.configMotionCruiseVelocity(80);
        arm.configMotionAcceleration(40);

    }

    public void setPower(double power) {
        arm.set(ControlMode.PercentOutput, power);
    }

    public void climberToAngle() {
        double home = arm.getSelectedSensorPosition();

        arm.configMotionCruiseVelocity(4000);
        arm.configMotionAcceleration(2000);
        arm.set(ControlMode.MotionMagic, (home + ClimberConstants.kTicksToRungAngle));

    }

    public void climberToVertical() {
        double home = arm.getSelectedSensorPosition();

        arm.configMotionCruiseVelocity(4000);
        arm.configMotionAcceleration(2000);
        arm.set(ControlMode.MotionMagic, (home + ClimberConstants.kTicksToVertical));
    }

    public void moveClimber(double ticksToTarget) {
        arm.set(ControlMode.MotionMagic, arm.getSelectedSensorPosition() + ticksToTarget);
    } 

}