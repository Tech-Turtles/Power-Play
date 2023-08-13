package org.firstinspires.ftc.teamcode.Utility.Swerve.hardware;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class SwerveAbsoluteEncoder {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;

    public SwerveAbsoluteEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public SwerveAbsoluteEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }
    public SwerveAbsoluteEncoder zero(double offset){
        this.offset = offset;
        return this;
    }
    public SwerveAbsoluteEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }
    public boolean getInverted() {
        return inverted;
    }

    private double pastPosition = 1;
    public double getCurrentPosition() {
        double pos = (1 - (encoder.getVoltage() / analogRange)) * 360;
        //checks for crazy values when the encoder is close to zero
        return pos;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }
}
