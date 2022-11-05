package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Configuration {

    public static final double deadzone = 0.49;

    public static double INTAKE_POS = 280;
    public static double LOW_POS = 270;
    public static double MEDIUM_POS = 525;
    public static double HIGH_POS = 925;

    public enum ServoPosition {
        START(0.04, 0.35),
        HOLD(0.14, 0.44),
        INTERMEDIARY(0.3, 0.6),
        INTAKE(0.5, 0.8);
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