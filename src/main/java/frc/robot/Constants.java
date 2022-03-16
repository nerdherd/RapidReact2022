package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        public static final double kDriveAlpha = 0.11765;
        public static final double kDriveOneMinusAlpha = 0.88235;

        public static final int kRightMasterTalonID = 30;
        public static final int kLeftMasterTalonID = 16;
    
        public static final int kRightSlaveTalonID = 31;
        public static final int kLeftSlaveTalonID = 17;

        public static final int kClimbMasterTalonID = 24;
        public static final int kClimbSlaveTalonID = 25;
        
        public static final int kEverybotArm = 15;

        public static final int kEverybotIntake = 14;
    }
    public static final class EverybotConstants {
        public static final double kHighAngle = -20073.00;
        public static final double kLowAngle = 0;
        public static final double kGravityAngle = -19500.00;
        public static final double kHighAngleThreshold = 0;
        public static final double kLowAngleThreshold = 0;

        public static final double kEverybotArmkP = 1.0;
        public static final double kEverybotArmkD = 0.0002;
        public static final double kEverybotArmFF = 0.0;

        public static final double kEverybotUpVoltage = 0.7;
        public static final double kEverybotDownVoltage = 0.2;
    
        public static final double kEverybotIntake = 0.5;
        public static final double kEverybotOuttake = 0.6;
        public static final double kEverybotAutoOuttake = 1.0;
    
        public static final double kEverybotClimberUp = 0.2;
        public static final double kEverybotClimberDown = 0.5;
        public static final double kEverybotClimberHigh = 0;
        public static final double kEverybotClimberLow = 0;

        public static final double kEverybotClimberkP = 0.01;
        public static final double kEverybotClimberkI = 0;
        public static final double kEverybotClimberkD = 0;
        public static final double kEverybotClimberkF = 0.188;
        
        public static final double kTicksToLowRung = -83361;
        public static final double kTicksToMidRung = 40604;
        public static final double kTicksToHome = 43757;
    }
    public static final class EverybotMotionMagicConstants {
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
    }  
    public static final class ArmConstants {
        public static double armKp = 1.0;
        public static double armKd = 0.0002;
    }
}
