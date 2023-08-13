package org.firstinspires.ftc.teamcode.Utility.Swerve;

import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Math.controller.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Math.kinematics.SwerveModulePosition;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.SwerveModuleConfiguration;
import org.firstinspires.ftc.teamcode.Utility.Swerve.hardware.AngleServo;
import org.firstinspires.ftc.teamcode.Utility.Swerve.hardware.DriveMotor;
import org.firstinspires.ftc.teamcode.Utility.Swerve.hardware.SwerveAbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Utility.Swerve.hardware.SwerveMotor;
import org.firstinspires.ftc.teamcode.Utility.Swerve.hardware.SwerveServo;

/**
 * The Swerve Module class which represents and controls Swerve Modules for the swerve drive.
 */
public class SwerveModule {

    /**
     * Swerve module configuration options.
     */
    public final  SwerveModuleConfiguration configuration;
    /**
     * Angle offset from the absolute encoder.
     */
    private final double                    angleOffset;
    /**
     * Swerve Motor.
     */
    private final SwerveMotor driveMotor;
    /**
     * Swerve Motor.
     */
    private final SwerveServo angleServo;
    /**
     * Absolute encoder for swerve drive.
     */
    private final SwerveAbsoluteEncoder absoluteEncoder;
    /**
     * Module number for kinematics, usually 0 to 3. front left -> front right -> back left -> back right.
     */
    public        int                       moduleNumber;
    /**
     * Feedforward for drive motor during closed loop control.
     */
    public SimpleMotorFeedforward feedforward;
    /**
     * Last swerve module state applied.
     */
    public        SwerveModuleState2        lastState;

    /**
     * Construct the swerve module and initialize the swerve module motors and absolute encoder.
     *
     * @param moduleNumber        Module number for kinematics.
     * @param moduleConfiguration Module constants containing CAN ID's and offsets.
     */
    public SwerveModule(int moduleNumber, SwerveModuleConfiguration moduleConfiguration) {
        //    angle = 0;
        //    speed = 0;
        //    omega = 0;
        //    fakePos = 0;
        this.moduleNumber = moduleNumber;
        configuration = moduleConfiguration;
        angleOffset = moduleConfiguration.angleOffset;

        // Initialize Feedforward for drive motor.
        feedforward = configuration.createDriveFeedforward();

        // Create motors from configuration and reset them to defaults.
        angleServo = new AngleServo(ContinuousServo.FRONT_LEFT);
        angleServo.setServo(moduleConfiguration.angleServo);
        angleServo.setEncoder(moduleConfiguration.absoluteEncoder);
        driveMotor = new DriveMotor(Motors.FRONT_LEFT);
        driveMotor.setMotor(moduleConfiguration.driveMotor);

        // Config angle encoders
        absoluteEncoder = moduleConfiguration.absoluteEncoder;

        lastState = getState();
    }

    /**
     * Set the desired state of the swerve module. <br /><b>WARNING: If you are not using one of the functions from
     * {@link SwerveDrive} you may screw up </b>
     *
     * @param desiredState Desired swerve module state.
     * @param isOpenLoop   Whether to use open loop (direct percent) or direct velocity control.
     * @param force        Disables optimizations that prevent movement in the angle motor and forces the desired state
     *                     onto the swerve module.
     */
    public void setDesiredState(SwerveModuleState2 desiredState, boolean isOpenLoop, boolean force) {
        desiredState = SwerveModuleState2.optimize(desiredState,
                Rotation2d.fromDegrees(getAbsolutePosition()),
                lastState,
                0.0);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / configuration.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            if (desiredState.speedMetersPerSecond != lastState.speedMetersPerSecond) {
                double velocity = desiredState.speedMetersPerSecond;
                driveMotor.setReference(velocity, feedforward.calculate(velocity));
            }
        }

        // If we are forcing the angle
        if (!force) {
            // Prevents module rotation if speed is less than 1%
            SwerveMath.antiJitter(desiredState, lastState, Math.min(configuration.maxSpeed, 4));
        }

//        if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
//        {
//            SmartDashboard.putNumber("Module[" + configuration.name + "] Speed Setpoint:", desiredState.speedMetersPerSecond);
//            SmartDashboard.putNumber("Module[" + configuration.name + "] Angle Setpoint:", desiredState.angle.getDegrees());
//            SmartDashboard.putNumber("Module[" + configuration.name + "] Omega:",
//                    Math.toDegrees(desiredState.omegaRadPerSecond));
//        }

        // Prevent module rotation if angle is the same as the previous angle.
        if (desiredState.angle != lastState.angle) {
            double moduleFF = desiredState.omegaRadPerSecond * configuration.moduleSteerFFCL;
            angleServo.setReference(desiredState.angle.getDegrees(), moduleFF);
        } else {
            angleServo.set(0.0);
        }

        lastState = desiredState;
    }

    /**
     * Set the angle for the module.
     *
     * @param angle Angle in degrees.
     */
    public void setAngle(double angle) {
        angleServo.setReference(angle, 0);
        lastState.angle = Rotation2d.fromDegrees(angle);
    }

    /**
     * Get the Swerve Module state.
     *
     * @return Current SwerveModule state.
     */
    public SwerveModuleState2 getState() {
        double     velocity;
        Rotation2d azimuth;
        double     omega;
        velocity = driveMotor.getVelocity();
        azimuth = Rotation2d.fromDegrees(angleServo.getPosition());
        omega = Math.toRadians(angleServo.getVelocity());
        return new SwerveModuleState2(velocity, azimuth, omega);
    }

    /**
     * Get the position of the swerve module.
     *
     * @return {@link SwerveModulePosition} of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        double     position;
        Rotation2d azimuth;
        position = driveMotor.getPosition();
        azimuth = Rotation2d.fromDegrees(getAbsolutePosition());
        return new SwerveModulePosition(position, azimuth);
    }

    /**
     * Get the absolute position.
     *
     * @return Absolute encoder angle in degrees in the range [0, 360).
     */
    public double getAbsolutePosition() {
        double angle = 0;
        if (absoluteEncoder != null)
        {
            angle = absoluteEncoder.getCurrentPosition() - angleOffset;
        }
        angle %= 360;
        if (angle < 0.0)
        {
            angle += 360;
        }

        return angle;
    }

    public SwerveServo getAngleServo() {
        return angleServo;
    }

    public SwerveMotor getDriveMotor() {
        return driveMotor;
    }

    /**
     * Fetch the {@link SwerveModuleConfiguration} for the {@link SwerveModule} with the parsed configurations.
     *
     * @return {@link SwerveModuleConfiguration} for the {@link SwerveModule}.
     */
    public SwerveModuleConfiguration getConfiguration() {
        return configuration;
    }
}
