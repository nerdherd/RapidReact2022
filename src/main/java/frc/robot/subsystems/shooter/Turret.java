package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NerdyMath;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    
    private TalonFX frontFlywheelFalcon;
    private TalonFX backFlywheelFalcon;
    private TalonFX hoodMotor;
    private TalonFX baseMotor;

    private double hoodLimitLower, hoodLimitUpper;

    public Turret() {
        this.frontFlywheelFalcon = new TalonFX(TurretConstants.kFrontFlywheelFalconID);
        this.backFlywheelFalcon = new TalonFX(TurretConstants.kBackFlywheelFalconID);
        this.hoodMotor = new TalonFX(TurretConstants.kHoodMotorID);
        this.baseMotor = new TalonFX(TurretConstants.kBaseMotorID);

        hoodLimitLower = NerdyMath.ticksToAngle(TurretConstants.kHoodLowerLimitTicks);
        hoodLimitUpper = NerdyMath.ticksToAngle(TurretConstants.kHoodUpperLimitTicks);
    }

    public void setVelocity(double velocity) {
        backFlywheelFalcon.set(ControlMode.Velocity, velocity);
        
        // Get front flywheel to spin at 1/4 the speed
        frontFlywheelFalcon.set(ControlMode.Velocity, velocity 
                                * (1/TurretConstants.kBackFlywheelGearRatio) 
                                * 0.25);
    }

    public void setVelocityZero() {
        setVelocity(0);
    }

    public void turnToBaseAngle(double angle) {
        double currentAngle = baseMotor.getSelectedSensorPosition() / TurretConstants.kBaseTicksPerRadian;
        double offsetAngle = TurretConstants.kBaseTicksPerRadian * constrainAngleBase(angle);
        double targetAngle = currentAngle + offsetAngle;
        baseMotor.set(ControlMode.MotionMagic, targetAngle);
    }

    public void turnToHoodAngle(double angle) {
        hoodMotor.set(
            ControlMode.MotionMagic, 
            TurretConstants.kHoodTicksPerRadian * 
            NerdyMath.clamp(angle, hoodLimitLower, hoodLimitUpper)
            );
    }

    private double constrainAngleBase(double rawAngle) {
        double newAngle = rawAngle % 360;
        if (newAngle < 0) {
            newAngle += 360;
        }

        return newAngle - 180;
    }

}