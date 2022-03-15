package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import frc.robot.Constants.EverybotMotionMagicConstants;

public class EverybotArmMotionMagic extends TrapezoidProfileSubsystem {
    private final ArmFeedforward feedforward = new ArmFeedforward(
        EverybotMotionMagicConstants.kArmStaticGain, 
        EverybotMotionMagicConstants.kArmGravityGain,
        EverybotMotionMagicConstants.kArmVelocityGain);

    public TalonFX arm;

    public EverybotArmMotionMagic() {
        super(
            new TrapezoidProfile.Constraints(
                EverybotMotionMagicConstants.kArmMaxVelocity, 
                EverybotMotionMagicConstants.kArmMaxAcceleration)
        );

        arm = new TalonFX(15);
        arm.configMotionAcceleration(EverybotMotionMagicConstants.kArmMotionAccel);
        arm.configMotionCruiseVelocity(EverybotMotionMagicConstants.kArmMotionCruiseVel);
        arm.configNeutralDeadband(EverybotMotionMagicConstants.kArmDeadband);
        arm.config_kP(0, EverybotMotionMagicConstants.kArmP);
        arm.config_kD(0, EverybotMotionMagicConstants.kArmD);
    }

    public double getPosition() {
        return arm.getSelectedSensorPosition();
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        double ff = feedforward.calculate(setpoint.position, setpoint.velocity);
        arm.config_kF(0, ff);
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" EHeight Current ", arm.getSupplyCurrent());
        SmartDashboard.putNumber(" EHeight Voltage ", arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" EHeight Position ", arm.getSelectedSensorPosition());
    }

    public double angleToTicks(double angle) {
        return (angle - EverybotMotionMagicConstants.kArmAngleOffset) 
            / EverybotMotionMagicConstants.kArmAngleRatio;
    }

    public void setPositionMotionMagic(double angle) {
        if (Math.abs(arm.getSelectedSensorVelocity()) <= EverybotMotionMagicConstants.kArmStaticFrictionDeadband) {
            arm.set(ControlMode.MotionMagic, angleToTicks(angle), DemandType.ArbitraryFeedForward, 
              getFFIfNotMoving(angle - getAngle()));
          } else {
            arm.set(ControlMode.MotionMagic, angleToTicks(angle), DemandType.ArbitraryFeedForward, 
              getFFIfMoving());
        }
    }

    public double getAngle() {
        return (EverybotMotionMagicConstants.kArmAngleRatio * arm.getSelectedSensorPosition()) 
        + EverybotMotionMagicConstants.kArmAngleRatio;
    }

    public double getFFIfNotMoving(double error) {
        double sign = Math.signum(error);
        return EverybotMotionMagicConstants.kArmGravityFF * Math.cos(degreesToRadians(getAngle())) + 
          sign * EverybotMotionMagicConstants.kArmStaticFF;
    }

    public double getFFIfMoving() {
        return EverybotMotionMagicConstants.kArmGravityFF * Math.cos(degreesToRadians(getAngle()));
    }

    public double degreesToRadians(double deg) {
		return deg * Math.PI / 180;
    }

    public void resetElevatorEncoder() {
        arm.setSelectedSensorPosition(0);
    }


}
