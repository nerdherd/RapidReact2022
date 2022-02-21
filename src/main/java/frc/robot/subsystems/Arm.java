package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ClimberConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class Arm extends TrapezoidProfileSubsystem {
    
    public static TalonSRX arm;
    public final ArmFeedforward feedforward = new ArmFeedforward(ClimberConstants.kArmStaticGain, ClimberConstants.kArmGravityGain, ClimberConstants.kArmVelocityGain);

    public double angleOffset = ClimberConstants.kAngleOffset;
    public double angleRatio = ClimberConstants.kAngleRatio;
    public double gravityFF = ClimberConstants.kGravityFF;
    public double staticFF = ClimberConstants.kStaticFF;

    public Arm(double angleOffset, double angleRatio, double gravityFF, double staticFF) {
        super(
            new TrapezoidProfile.Constraints(ClimberConstants.kArmMaxVelocity, ClimberConstants.kArmMaxAcceleration)
            // TODO: config arm pid here
        );
        arm = new TalonSRX(15);

        this.angleOffset = angleOffset;
        this.angleRatio = angleRatio;
        this.gravityFF = gravityFF;
        this.staticFF = staticFF;
    }

    public static double getPosition() {
        return arm.getSelectedSensorPosition();
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        double ff = feedforward.calculate(setpoint.position, setpoint.velocity);
        // TODO: use feedforward
    }

    public static double angleToTicks(double angle, double diameter, double drivetrainWidth) {
        return Math.toRadians(angle) * 0.5 * (drivetrainWidth / Math.PI / diameter * 4096);
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Arm Current ", arm.getSupplyCurrent());
        SmartDashboard.putNumber(" Arm Voltage ", arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Arm Elevator ", arm.getSelectedSensorPosition());
    }

    public double angleToTicks(double angle) {
        return (angle - angleOffset) / angleRatio;
    }

    public void setPositionMotionMagic(double angle) {
        if (Math.abs(arm.getSelectedSensorVelocity()) <= ClimberConstants.kStaticFrictionDeadband) {
            arm.set(ControlMode.MotionMagic, angleToTicks(angle), DemandType.ArbitraryFeedForward, 
              getFFIfNotMoving(angle - getAngle()));
          } else {
            arm.set(ControlMode.MotionMagic, angleToTicks(angle), DemandType.ArbitraryFeedForward, 
              getFFIfMoving());
        }
    }

    public double getMotorOutputVoltage() {
        return arm.getMotorOutputVoltage();
    }

    public double getAngle() {
        return (angleRatio * arm.getSelectedSensorPosition()) + angleRatio;
    }

    public double getFFIfNotMoving(double error) {
        double sign = Math.signum(error);
        return gravityFF * Math.cos(degreesToRadians(getAngle())) + 
          sign * staticFF;
    }

    public double getFFIfMoving() {
        return gravityFF * Math.cos(degreesToRadians(getAngle()));
    }

    public static double degreesToRadians(double deg) {
		return deg * Math.PI / 180;
    }
}
