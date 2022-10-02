package frc.robot;

public class MathUtil {
    public static double ticksToAngle(double ticks, double ticksPerRotation) {
        return ticks * 360 / ticksPerRotation;
    }

    public static double ticksToAngle(double ticks) {
        return ticks * 360 / 4096;
    }

    public static double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }

    public static double radiansToDegrees(double rad) {
        return rad * 180 / Math.PI;
    }

    public static double angleToTicks(double angle, double ticksPerRotation) {
        return angle * ticksPerRotation / 360;
    }
}