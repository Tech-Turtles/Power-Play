package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Configuration {

    public static final double deadzone = 0.49;

    public static final double TURRET_TICKS_PER_DEGREE = 7.155;

    public static double INTAKE_POS = 280;
    public static double LOW_POS = 170;
    public static double MEDIUM_POS = 525;
    public static double HIGH_POS = 800;

    public static double CLAW_OPEN = 1.0;
    public static double CLAW_ALIGN = 0.5;
    public static double CLAW_CLOSED = 0.0;

    // Measured from normal with positive to the right
    public static double LEFT_CAMERA_DEG = 18.0;
    public static double RIGHT_CAMERA_DEG = -20.0;
    public static double CAMERA_DISTANCE_IN = 9.375;

    public static double CLAW_DISTANCE_IN = 1.0;
    public static double ARM_CONE_OFFSET_IN = 1.5;

    public static int DEFAULT_PARK_ORDINAL = 1;

    public enum ServoPosition {
        START(0.85, 0.85),
        HOLD(0.8, 0.8),
        DAMPEN(0.7, 0.7),
        PLACE(0.67, 0.67),
        INTERMEDIARY(0.3, 0.3),
        INTAKE(0.3, 0.3),
        LOW_INTAKE(0.2, 0.2),
        NONE(0.0, 0.0);
        private final double left;
        private final double right;
        ServoPosition(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return left;
        }

        public double getRight() {
            return right;
        }
    }
}