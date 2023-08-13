package org.firstinspires.ftc.teamcode.Utility.Swerve.hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.PIDFConfig;

/**
 * Swerve motor abstraction which defines a standard interface for motors within a swerve module.
 */
public abstract class SwerveServo {

    protected final ContinuousServo servoConfig;
    public double pid;

    SwerveServo(ContinuousServo servo) {
        this.servoConfig = servo;
    }

    /**
     * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
     *
     * @param config Configuration class holding the PIDF values.
     */
    public abstract void configurePIDF(PIDFConfig config);

    /**
     * Configure the PID wrapping for the position closed loop controller.
     *
     * @param minInput Minimum PID input.
     * @param maxInput Maximum PID input.
     */
    public abstract void configurePIDWrapping(double minInput, double maxInput);

    /**
     * Set the motor to be inverted.
     *
     * @param inverted State of inversion.
     */
    public abstract void setInverted(boolean inverted);

    /**
     * Set the percentage output.
     *
     * @param percentOutput percent out for the motor controller.
     */
    public abstract void set(double percentOutput);

    /**
     * Set the closed loop PID controller reference point.
     *
     * @param setpoint    Setpoint in meters per second or angle in degrees.
     * @param feedforward Feedforward in volt-meter-per-second or kV.
     */
    public abstract void setReference(double setpoint, double feedforward);

    /**
     * Set the closed loop PID controller reference point.
     *
     * @param setpoint    Setpoint in meters per second or angle in degrees.
     * @param feedforward Feedforward in volt-meter-per-second or kV.
     * @param position    Only used on the angle motor, the position of the motor in degrees.
     */
    public abstract void setReference(double setpoint, double feedforward, double position);

    /**
     * Get the velocity of the integrated encoder.
     *
     * @return velocity in meters per second or degrees per second.
     */
    public abstract double getVelocity();

    /**
     * Get the position of the integrated encoder.
     *
     * @return Position in meters or degrees.
     */
    public abstract double getPosition();

    /**
     * Set the integrated encoder position.
     *
     * @param position Integrated encoder position. Should be angle in degrees or meters per second.
     */
    public abstract void setPosition(double position);

    public ContinuousServo getServoConfig() {
        return servoConfig;
    }

    public abstract void setEncoder(SwerveAbsoluteEncoder encoder);

    /**
     * Set the servo object for the module.
     */
    public abstract void setServo(CRServoImplEx s);

    /**
     * Get the servo object from the module.
     *
     * @return Motor object.
     */
    public abstract CRServoImplEx getServo();
}

