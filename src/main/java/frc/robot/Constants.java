package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        public static final double kDriveAlpha = 0.11765;
        public static final double kDriveOneMinusAlpha = 0.88235;

        public static final int kRightMasterTalonID = 30;
        public static final int kLeftMasterTalonID = 16;
        public static final int kClimbMasterTalonID = 24;
    
        public static final int kRightSlaveTalonID = 31;
        public static final int kLeftSlaveTalonID = 17;
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
    }
}
