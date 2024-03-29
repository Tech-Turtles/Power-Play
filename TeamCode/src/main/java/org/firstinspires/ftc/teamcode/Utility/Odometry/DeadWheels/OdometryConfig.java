package org.firstinspires.ftc.teamcode.Utility.Odometry.DeadWheels;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.Navigation2D;

@Config
public class OdometryConfig {

    public static final double wheelDiameter = 60.0 * 0.0393701; // 60mm diameter, 7.417" circumference
    public static final double ticksPerRotation = 8192.0;

    public static double outerWheelDistance = 9.061;
    public static double outerWheelXPos = 0.0;
    public static double centerWheelXPos = -1.091;
    public static double strafeErrorPerDegrees = centerWheelXPos * 2.0 * Math.PI / 360.0;

    public static double STRAFE_ROTATION_ERROR_DEG_PER_INCH = -7.0 / 72.0; // should be final, won't update from dashboard
    public static double FWD_WHEEL_TOE_OUT_DEG = STRAFE_ROTATION_ERROR_DEG_PER_INCH * outerWheelDistance;

    public static double FWD_WHEEL_BIAS = 0f;
    // Degrees
    public static double CENTER_WHEEL_BIAS = 0f;

    // Nav2D of wheels.
    public static Navigation2D getLeftWheelPosition() {
        return new Navigation2D(outerWheelXPos, outerWheelDistance / 2.0, Math.toRadians(FWD_WHEEL_TOE_OUT_DEG / 2.0 + FWD_WHEEL_BIAS));
    }

    public static Navigation2D getCenterWheelPosition() {
        return new Navigation2D(centerWheelXPos,0,Math.toRadians(90.0 + CENTER_WHEEL_BIAS));
    }

    public static Navigation2D getRightWheelPosition() {
        return new Navigation2D(outerWheelXPos,-outerWheelDistance/2.0,Math.toRadians(-FWD_WHEEL_TOE_OUT_DEG / 2.0 + FWD_WHEEL_BIAS));
    }


    public static double inchesFromTicks(double ticks) {
        return (ticks / ticksPerRotation) * (wheelDiameter * Math.PI);
    }

    public static double ticksFromInches(double inches) {
        return inches * ticksPerRotation / (wheelDiameter * Math.PI);
    }
}