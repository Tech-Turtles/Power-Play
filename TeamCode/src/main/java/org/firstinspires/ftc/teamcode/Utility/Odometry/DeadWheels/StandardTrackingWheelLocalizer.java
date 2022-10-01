package org.firstinspires.ftc.teamcode.Utility.Odometry.DeadWheels;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.Frame2D;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import java.util.Arrays;
import java.util.List;

import androidx.annotation.NonNull;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */

public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer implements Localizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.49606; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 6.8; // in; offset of the lateral wheel

    private RobotHardware opmode;

    public StandardTrackingWheelLocalizer(RobotHardware opmode) {
        super(Arrays.asList(
                toPose2dFromNav2d(OdometryConfig.getLeftWheelPosition()), // left
                toPose2dFromNav2d(OdometryConfig.getRightWheelPosition()), // right
                toPose2dFromNav2d(OdometryConfig.getCenterWheelPosition()) // center
        ));

        this.opmode = opmode;
    }

    public static double encoderTicksToInches(int ticks) {
        return OdometryConfig.inchesFromTicks(ticks);
    }

    public static Pose2d toPose2dFromNav2d(Navigation2D navigation2D) {
        return new Pose2d(navigation2D.x, navigation2D.y, navigation2D.theta);
    }

    public static Navigation2D toNav2dFromPose2d(Pose2d pose2d) {
        return new Navigation2D(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(opmode.motorUtility.getEncoderValue(Motors.FRONT_LEFT)),
                encoderTicksToInches(opmode.motorUtility.getEncoderValue(Motors.BACK_LEFT)),
                encoderTicksToInches(opmode.motorUtility.getEncoderValue(Motors.BACK_RIGHT))
        );
    }

    @Override
    public void update(RobotHardware robotHardware) {
        update();
    }


    private double previousHeading = 0;
    private double headingChange = 0;
    private double headingCompensation = 0;
    private double heading_deg = 0;
    // Unwrap Angle to be +- infinity
    @Override
    public Navigation2D getCurrentPosition() {
        Navigation2D currentPosition = toNav2dFromPose2d(getPoseEstimate());
        heading_deg = Math.toDegrees(currentPosition.theta);

        headingChange = heading_deg - previousHeading;
        previousHeading = Math.toDegrees(currentPosition.theta);
        headingCompensation = headingChange > 180 ? headingCompensation - 360 : (headingChange < -180 ? headingCompensation + 360 : headingCompensation);

        currentPosition.addInPlace(0,0,Math.toRadians(headingCompensation));
        return currentPosition;
    }

    @Override
    public void setCurrentPosition(Navigation2D currentPosition) {
        this.setPoseEstimate(toPose2dFromNav2d(currentPosition));
    }

    @Override
    public MecanumNavigation.Frame2D getRobotFrame() {
        return new Frame2D(getCurrentPosition());
    }
}
