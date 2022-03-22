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

        public static final double kEverybotUpVoltage = 0.7;
        public static final double kEverybotDownVoltage = 0.2;
    
        public static final double kEverybotIntake = 0.5;
        public static final double kEverybotOuttake = 0.6;
        public static final double kEverybotAutoOuttake = 1.0;
    
        public static final double kEverybotClimberUp = 0.2;
        public static final double kEverybotClimberDown = 0.5;
        public static final double kEverybotClimberHigh = 0;
        public static final double kEverybotClimberLow = 0;
        
        public static final double kEverybotClimberkP = 0.02; //0.01
        public static final double kEverybotClimberkI = 0;
        public static final double kEverybotClimberkD = 0;
        public static final double kEverybotClimberkF = 0.094;
        
        public static final double kTicksToLowRung = -102862; // 32645;
        public static final double kTicksToClimbLowRung = 52035;
        public static final double kTicksToHome = 43757;
    }
    
    public static final class ClimberConstants {
        public static final double kArmkP = 0.04;
        public static final double kArmkI = 0;
        public static final double kArmkD = 0;
        public static final double kArmkF = 0; 

        public static final double kArmStaticGain = 0.0;
        public static final double kArmGravityGain = 0.11;
        public static final double kArmVelocityGain = 0;

        public static final double kArmGravityFF = 0.11;
        public static final double kArmStaticFF = 0.0;

        public static final double kArmCruiseVelocity = 40;
        public static final double kArmMotionAcceleration = 20;
        public static final double kArmMaxVelocity = 54.0;
        public static final double kArmMaxAcceleration = 54.0;
        public static final double kArmDeadband = 0.0004;
        public static final double kArmStaticFrictionDeadband = 5.0;

        public static final double kArmAngleOffset = 60;
        public static final double kArmAngleLength = 1.0 / 4096 * 360 * 29;

        public static final double kTicksToRungAngle = -200; //Actual ticks 620, not resetting encoder position properly
        public static final double kTicksToVertical = -60;
    }
    
}
