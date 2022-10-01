package org.firstinspires.ftc.teamcode.Utility.Mecanum;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utility.Odometry.DeadWheels.Localizer;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

/**
 * Created by Ashley on 12/18/2017.
 */

public class AutoDrive {

    private RobotHardware opMode;
    private MecanumNavigation mecanumNavigation;
    private Localizer localizer; // Could be either MecanumNavigation or OdometryLocalizer.
    public MecanumNavigation.Navigation2D lastTargetPosition = new MecanumNavigation.Navigation2D(0,0,0);


    public AutoDrive(RobotHardware opMode, MecanumNavigation mecanumNavigation, Localizer localizer) {
        this.opMode = opMode;
        this.mecanumNavigation = mecanumNavigation;
        this.localizer = localizer;
    }

    /**
     * Drive to position. Simple calculation, drives in single rotational arc.
     * In general, not the most efficient path.
     * @param targetPosition
     * @param rate
     * @return boolean, true if currentPosition is near targetPosition.
     */
    public boolean driveToPosition(Navigation2D targetPosition, double rate) {
        lastTargetPosition = targetPosition;
        double distanceThresholdInches = 1;
        double angleThresholdRadians = 2 * (2*Math.PI/180);
        rate = Range.clip(rate,0,1);
        Navigation2D currentPosition = localizer.getCurrentPosition();
        Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);

        // Not near enough to target position
        if ( Math.abs(deltaPosition.x) > distanceThresholdInches ||
                Math.abs(deltaPosition.y) > distanceThresholdInches ||
                Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            Mecanum.Wheels wheels = MecanumNavigation.deltaWheelsFromPosition(localizer.getCurrentPosition(),targetPosition,mecanumNavigation.driveTrainMecanum);
            wheels.scaleWheelPower(rate);
            opMode.motorUtility.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Near enough to target position
            opMode.motorUtility.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }
    }

    public boolean rotateThenDriveToPosition(Navigation2D targetPosition, double rate) {
        lastTargetPosition = targetPosition;
        double distanceThresholdInches = 0.5;
        double angleThresholdRadians = 2.0 * (Math.PI/180.0);
        rate = Range.clip(rate,0,1);
        Navigation2D currentPosition = localizer.getCurrentPosition();
        Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);
        double deltaDistance = Math.sqrt( Math.pow(deltaPosition.x,2) + Math.pow(deltaPosition.y,2));

        double rateScale;
        // Not near enough to target position
        if ( Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            Navigation2D rotationTarget = currentPosition.copy();
            rotationTarget.theta = targetPosition.theta; // Only rotate to the target at first.
            Mecanum.Wheels wheels = MecanumNavigation.deltaWheelsFromPosition(
                    localizer.getCurrentPosition(),rotationTarget,mecanumNavigation.driveTrainMecanum);
            rateScale = rampDown(Math.abs(deltaPosition.theta)*(180/Math.PI), 50, 0.8, 0.1);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.motorUtility.setDriveForMecanumWheels(wheels);
            return false;
        } else if (Math.abs(deltaPosition.x) > distanceThresholdInches ||
                   Math.abs(deltaPosition.y) > distanceThresholdInches) {

            Mecanum.Wheels wheels = MecanumNavigation.deltaWheelsFromPosition(localizer.getCurrentPosition(),targetPosition,mecanumNavigation.driveTrainMecanum);
            rateScale = rampDown(deltaDistance, 10, 1, 0.05);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.motorUtility.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Near enough to target position
            opMode.motorUtility.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }

    }

    public boolean driveToPositionTranslateOnly(Navigation2D targetPosition, double rate) {
        return driveToPositionTranslateOnly(targetPosition, rate,0.05, 0.5);
    }

    public boolean driveToPositionTranslateOnly(Navigation2D targetPosition, double rate, double minRate) {
        return driveToPositionTranslateOnly(targetPosition, rate, minRate, 0.5);
    }

    public boolean driveToPositionTranslateOnly(Navigation2D targetPosition, double rate, double minRate, double distanceThresholdInches) {
        lastTargetPosition = targetPosition;
        double angleThresholdRadians = 2.0 * (Math.PI/180.0);
        rate = Range.clip(rate,0,1);
        Navigation2D currentPosition = localizer.getCurrentPosition();
        Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);
        double deltaDistance = Math.sqrt( Math.pow(deltaPosition.x,2) + Math.pow(deltaPosition.y,2));

        double rateScale;
        // Not near enough to target position
        if (Math.abs(deltaPosition.x) > distanceThresholdInches ||
                Math.abs(deltaPosition.y) > distanceThresholdInches ||
                Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            Navigation2D translationTarget = (Navigation2D)currentPosition.clone();
            translationTarget.x = targetPosition.x;
            translationTarget.y = targetPosition.y;
            Mecanum.Wheels wheels = MecanumNavigation.deltaWheelsFromPosition(localizer.getCurrentPosition(),targetPosition,mecanumNavigation.driveTrainMecanum);
            rateScale = rampDown(deltaDistance, 10, 1, minRate);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.motorUtility.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Near enough to target position
            opMode.motorUtility.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }
    }

    static public double rampDown(double errSignal, double signalRampDownThreshold, double maxRatio, double minRatio) {
        // Error Check: Swap values to ensure max >= min
        if (maxRatio < minRatio) {
            double temp = maxRatio;
            maxRatio = minRatio;
            minRatio = temp;
        }

        double output;
        double standardOutput = 1.0;

        if (errSignal > signalRampDownThreshold)
            output = standardOutput;
        else
            output = standardOutput * (minRatio + (maxRatio - minRatio) * errSignal/signalRampDownThreshold);

        return Range.clip(Math.abs(output),0,1);
    }

    public boolean driveMotorToPos (Motors motor, int targetTicks, double power, int rampDistanceTicks) {
        int poweredDistance = 5;
        int arrivedDistance = 50;
        double maxRampPower = 1.0;
        double minRampPower = 0.0;

        power = Range.clip(Math.abs(power), 0, 1);
        int errorSignal = opMode.motorUtility.getEncoderValue(motor) - targetTicks;
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = rampDown(Math.abs(errorSignal), rampDistanceTicks, maxRampPower, minRampPower);

        if (Math.abs(errorSignal) >= poweredDistance)
            opMode.motorUtility.setPower(motor, direction * power * rampDownRatio);
        else
            opMode.motorUtility.setPower(motor, 0);

        return Math.abs(errorSignal) <= arrivedDistance;
    }

    public boolean driveMotorToPos (Motors motor, int targetTicks, double power) {
        return driveMotorToPos (motor, targetTicks, power, 400);
    }
}
