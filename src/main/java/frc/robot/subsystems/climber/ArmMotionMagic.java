package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ClimberConstants;

public class ArmMotionMagic extends TrapezoidProfileSubsystem {
    private final ArmFeedforward feedforward = new ArmFeedforward(
        ClimberConstants.kArmStaticGain, 
        ClimberConstants.kArmGravityGain, 
        ClimberConstants.kArmVelocityGain);

    public TalonSRX arm;

    public ArmMotionMagic() {
        super(
            new TrapezoidProfile.Constraints(
                ClimberConstants.kArmMaxVelocity,
                ClimberConstants.kArmMaxAcceleration)
        );

        arm = new TalonSRX(7);
        arm.configMotionAcceleration(ClimberConstants.kArmMotionAcceleration);
        arm.configMotionCruiseVelocity(ClimberConstants.kArmCruiseVelocity);
        arm.configNeutralDeadband(ClimberConstants.kArmDeadband);
        arm.config_kP(0, ClimberConstants.kArmkP);
        arm.config_kD(0, ClimberConstants.kArmkD);
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        double ff = feedforward.calculate(setpoint.position, setpoint.velocity);
        arm.config_kF(0, ff);
    }

    public void setPositionMotionMagic(double angle) {
        if (Math.abs(arm.getSelectedSensorVelocity()) <= ClimberConstants.kArmStaticFrictionDeadband) {
            getFFIfNotMoving(angle - getAngle());
        }
        else {
            arm.set(ControlMode.MotionMagic, angleToTicks(angle), DemandType.ArbitraryFeedForward, getFFIfMoving());
        }
    }

    public void resetClimbEncoder() {
        arm.setSelectedSensorPosition(0);
    }  

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Climber Position ", arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Voltage ", arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Climber Voltage ", arm.getSupplyCurrent());
    }

    public double getFFIfNotMoving(double error ){
        double sign = Math.signum(error);
        return (ClimberConstants.kArmGravityFF * Math.cos(degreesToRadians(getAngle()))
        + sign * ClimberConstants.kArmAngleRatio);
    }

    public double getFFIfMoving() {
        return ClimberConstants.kArmGravityFF * Math.cos(degreesToRadians(getAngle()));
    }

    public double getPosition() {
        return arm.getSelectedSensorPosition();
    }

    public double getAngle() {
        return (ClimberConstants.kArmAngleRatio * arm.getSelectedSensorPosition() 
        + ClimberConstants.kArmAngleRatio);
    }

    public double angleToTicks(double angle) {
        return (angle = ClimberConstants.kArmAngleOffset
            / ClimberConstants.kArmAngleRatio);
    }

    public double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }  

}
