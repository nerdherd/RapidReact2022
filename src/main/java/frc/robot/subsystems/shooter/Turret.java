package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    
    private TalonFX frontFlywheelFalcon;
    private TalonFX backFlywheelFalcon;
    private TalonFX hoodMotor;
    private TalonFX baseMotor;

    private double currentAngle;

    public Turret() {
        this.frontFlywheelFalcon = new TalonFX(TurretConstants.frontFlywheelFalconID);
        this.backFlywheelFalcon = new TalonFX(TurretConstants.backFlywheelFalconID);
        this.hoodMotor = new TalonFX(TurretConstants.hoodMotorID);
        this.baseMotor = new TalonFX(TurretConstants.baseMotorID);
    }

    public void setVelocity(double velocity) {
        backFlywheelFalcon.set(ControlMode.Velocity, velocity);
        
        // Get front flywheel to spin at 1/4 the speed
        frontFlywheelFalcon.set(ControlMode.Velocity, velocity 
                                * (1/TurretConstants.backFlywheelGearRatio) 
                                * 0.25);
    }

    public void setVelocityZero() {
        setVelocity(0);
    }

    public void turnToBaseAngle(double angle) {
        baseMotor.set(ControlMode.MotionMagic, TurretConstants.baseTicksPerRadian * angle);
    }

    public void turnToHoodAngle(double angle) {
        hoodMotor.set(ControlMode.MotionMagic, TurretConstants.hoodTicksPerRadian * angle);
    }
}