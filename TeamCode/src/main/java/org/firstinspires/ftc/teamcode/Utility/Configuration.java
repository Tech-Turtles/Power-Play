package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Configuration {

    public static final double deadzone = 0.49;
    public static int ARM_DRIVE_POS = 1300;
    public static int ARM_MIDDLE_POS = 800 + 300 + 125;
    public static int ARM_HIGH_POS = 1730;
    public static int ARM_LOW_POS = 350 + 75 + 100 + 100;
    public static int ARM_PICKUP_POS = 0;
    public static int ARM_MAX = 1800;
    public static double intakeExtend = 0.4;
    public static double intakeRetract = 0;

    public enum ServoPosition {
        INTAKE(0.0),
        CRADLE(0.2),
        DROP(1.0);
        private final double pos;
        ServoPosition(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }
}