package frc.robot.everybot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class EverybotHeight extends TrapezoidProfileSubsystem{
    
    public static final double kArmGoalAngle = 1.572; // Radians
    public static final double kArmVelocityToGoal = 0;
    public static final double kArmStaticFrictionDeadband = 5; // In ticks/decisecond, pulled from NerdyLib
    public static final double kArmDeadband = 0.004;
    public static final double kArmAngleRatio = 1./4096. * 360 * 12. / 22.; // (pulled from DeepSpace2019)
    public static final double kArmAngleOffset = 0; // Degrees (-15)
    
    // Feedforward Constants
    public static final double kArmStaticGain = 0;
    public static final double kArmGravityGain = 0; // https://www.chiefdelphi.com/t/motion-magic-with-an-arm/348667 (how to calculate cos gain)
    public static final double kArmVelocityGain = 0;
    public static final double kArmMaxVelocity = 540; // Max vel and max accel copied from motion magic vals
    public static final double kArmMaxAcceleration = 540;
    public static final double kArmStaticFF = 0.52;
    public static final double kArmGravityFF = 1.83;

    // Motion Constants (in sensor units per 100 ms)
    public static final double kArmMotionCruiseVel = 540; // Cruise velocity = slightly smaller than max velocity
    public static final double kArmMotionAccel = 540; // Run mechanism at 12 volts, take slope of velocity curve in talon units

    // PID
    public static final double kArmP = 4;
    public static final double kArmD = 0;    
    
    private final ArmFeedforward feedforward = new ArmFeedforward(
        kArmStaticGain, 
        kArmGravityGain,
        kArmVelocityGain);
    
    public static TalonFX arm;

    public EverybotHeight() {
        super(
            new TrapezoidProfile.Constraints(kArmMaxVelocity, kArmMaxAcceleration)
        );

        arm = new TalonFX(15);
        arm.configMotionAcceleration(kArmMotionAccel);
        arm.configMotionCruiseVelocity(kArmMotionCruiseVel);
        arm.configNeutralDeadband(kArmDeadband);
        arm.config_kP(0, kArmP);
        arm.config_kD(0, kArmD);
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
        return (angle - kArmAngleOffset) / kArmAngleRatio;
    }

    public void setPositionMotionMagic(double angle) {
        if (Math.abs(arm.getSelectedSensorVelocity()) <= kArmStaticFrictionDeadband) {
            arm.set(ControlMode.MotionMagic, angleToTicks(angle), DemandType.ArbitraryFeedForward, 
              getFFIfNotMoving(angle - getAngle()));
          } else {
            arm.set(ControlMode.MotionMagic, angleToTicks(angle), DemandType.ArbitraryFeedForward, 
              getFFIfMoving());
        }
    }

    public double getAngle() {
        return (kArmAngleRatio * arm.getSelectedSensorPosition()) + kArmAngleRatio;
    }

    public double getFFIfNotMoving(double error) {
        double sign = Math.signum(error);
        return kArmGravityFF * Math.cos(degreesToRadians(getAngle())) + 
          sign * kArmStaticFF;
    }

    public double getFFIfMoving() {
        return kArmGravityFF * Math.cos(degreesToRadians(getAngle()));
    }

    public double degreesToRadians(double deg) {
		return deg * Math.PI / 180;
    }

    public static void resetElevatorEncoder() {
        arm.setSelectedSensorPosition(0);
    }


}
