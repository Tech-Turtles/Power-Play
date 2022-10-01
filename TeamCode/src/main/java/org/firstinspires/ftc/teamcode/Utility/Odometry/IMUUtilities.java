package org.firstinspires.ftc.teamcode.Utility.Odometry;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import java.util.Locale;

/**
 * Created by Ashley on 1/18/2018.
 */

public class IMUUtilities {

    // Convenience numbers for when IMUUtilities is actually instantiated.
    public BNO055IMU imu;
    private double heading;
    public double roll;
    public double pitch;
    public double xAccel;
    public double yAccel;
    public double zAccel;

    protected RobotHardware opMode;
    protected Orientation angles;
    protected Acceleration gravity;
    protected Acceleration acceleration;

    protected BNO055IMU.SystemStatus imuSystemStatus;
    protected BNO055IMU.CalibrationStatus imuCalibrationStatus;

    private double lastUpdateSec = 0;
    private final double minUpdateDelay = 1.0;
    private final ImuMode imuMode;

    private static final StringBuilder builder = new StringBuilder();

    public enum ImuMode {
        FAST_HEADING_ONLY,
        SLOW_ALL_MEASUREMENTS,
    }


    // Default to Fast, Heading only mode when not specified.
    public IMUUtilities(RobotHardware opMode, String imu_name) {
        this(opMode,imu_name, ImuMode.FAST_HEADING_ONLY);
    }

    public IMUUtilities(RobotHardware opMode, String imu_name, ImuMode imuMode) {
        this.opMode = opMode;
        this.imuMode = imuMode;
        imu = initializeIMU(this.opMode, imu_name, imuMode);
        startIMU(imu);
    }

    private double previousHeading = 0;

    public void updateNow() {
        if (imu == null) return;

        lastUpdateSec = opMode.time;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        roll = angles.secondAngle;
        pitch = angles.thirdAngle;

        // Only grab non-heading info if running in slow mode.
        if(imuMode == ImuMode.SLOW_ALL_MEASUREMENTS) {
            imuSystemStatus = imu.getSystemStatus();
            imuCalibrationStatus = imu.getCalibrationStatus();
            gravity = imu.getGravity();

            acceleration = imu.getLinearAcceleration();
            xAccel = acceleration.xAccel;
            yAccel = acceleration.yAccel;
            zAccel = acceleration.zAccel;
        }

        // Unwrap compensated heading.
        double headingChange = heading - previousHeading;
        previousHeading = angles.firstAngle;
        headingCompensation = headingChange > 180 ? headingCompensation - 360 :
                (headingChange < -180 ? headingCompensation + 360 : headingCompensation);

    }

    public void update() {
        if (imu == null) {
            opMode.telemetry.addLine("IMU Missing");
            Log.e("IMUUtilities", "IMU Missing");
            return;
        }

        // Update rate is limited by minUpdateDelay to prevent too many costly operations.
        // Updating this data is quite expensive.
        // Seconds
        if (opMode.time - lastUpdateSec > minUpdateDelay) {
          updateNow();
        }
    }

    // How stale is our data?
    public double dataAge() {
      return opMode.time - lastUpdateSec;
    }

    public void displayTelemetry() {
        if (imu == null) return;
        builder.setLength(0);
        builder.append("---IMU Data---");

        switch (imuMode) {
            case SLOW_ALL_MEASUREMENTS:
                displayIMUTelemetry(imuSystemStatus, imuCalibrationStatus, angles, gravity, acceleration, opMode);
                break;
            case FAST_HEADING_ONLY:
                builder.append("\n")
                        .append("Heading").append(formatAngle(angles.angleUnit, angles.firstAngle)).append("\n")
                        .append("Roll").append(formatAngle(angles.angleUnit, angles.secondAngle)).append("\n")
                        .append("Pitch").append(formatAngle(angles.angleUnit, angles.thirdAngle)).append("\n");
        }

        builder.append("IMU Data Age: ").append(dataAge()).append("\n");
        opMode.telemetry.addLine(builder.toString());
    }

    // Static Functions

    /**
     * Use IMU config name to initialize and return a reference to the imu hardware.
     * @param opMode    Reference to opmode
     * @param imu_name  Configuration name for IMU
     * @return          Initialized IMU object
     */
    static public BNO055IMU initializeIMU(RobotHardware opMode, String imu_name, ImuMode imuMode) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = imuMode != ImuMode.FAST_HEADING_ONLY;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        BNO055IMU imu;
        try {
            imu = opMode.hardwareMap.get(BNO055IMU.class, imu_name);
            imu.initialize(parameters);
        } catch (Exception e) {
            imu = null;
            opMode.telemetry.addData("IMU Missing", imu_name);
        }
        return imu;
    }

    /**
     * Start IMU integration and logging.
     * @param imu   Reference to the IMU
     */
    static public void startIMU (BNO055IMU imu) {
        // If IMU is missing, do nothing.
        if (imu == null) {return;}
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    /**
     * Display telemetry data for the IMU
     */
    static public void displayIMUTelemetry(BNO055IMU.SystemStatus systemStatus,
                                           BNO055IMU.CalibrationStatus calibrationStatus,
                                           Orientation angles, Acceleration gravity, Acceleration acceleration,
                                           RobotHardware opMode) {
        builder.setLength(0);
        builder.append("---IMU Data---");

        builder.append("\n")
                .append("Status:").append(systemStatus.toShortString())
                .append("Calibration Status: ").append(calibrationStatus.toString());
        builder.append("\n")
                .append("Heading").append(formatAngle(angles.angleUnit, angles.firstAngle))
                .append("Roll").append(formatAngle(angles.angleUnit, angles.secondAngle))
                .append("Pitch").append(formatAngle(angles.angleUnit, angles.thirdAngle));
        builder.append("\n")
                .append("Grav").append(gravity.toString())
                .append("Mag").append(formatAcceleration(
                Math.sqrt(gravity.xAccel*gravity.xAccel
                        + gravity.yAccel*gravity.yAccel
                        + gravity.zAccel*gravity.zAccel)));
        builder.append("\n")
                .append("X Accel").append(formatAcceleration(acceleration.xAccel))
                .append("Y Accel").append(formatAcceleration(acceleration.yAccel))
                .append("Z Accel").append(formatAcceleration(acceleration.zAccel))
                .append("\n");
        opMode.telemetry.addLine(builder.toString());
    }



    static public void displayIMUTelemetry(final BNO055IMU imu, RobotHardware opMode) {
        // If IMU is missing, do nothing.
        if (imu == null) return;
        BNO055IMU.SystemStatus systemStatus = imu.getSystemStatus();
        BNO055IMU.CalibrationStatus calibrationStatus = imu.getCalibrationStatus();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = imu.getGravity();
        Acceleration acceleration = imu.getLinearAcceleration();

        displayIMUTelemetry(systemStatus, calibrationStatus, angles, gravity, acceleration, opMode);
    }


    static public class OrientationAngles {
        public double heading;
        public double roll;
        public double pitch;

        public OrientationAngles(double heading, double roll, double pitch) {
            this.heading = heading;
            this.roll = roll;
            this.pitch = pitch;
        }
    }


    /**
     * Return object with robot heading, roll, and pitch in degrees.
     * @param imu   Reference to IMU
     * @return OrientationAngles object
     */
    static public OrientationAngles getOrientation(BNO055IMU imu) {
        // If IMU is missing, do nothing.
        if (imu == null) {
            return new OrientationAngles(-999, -999, -999);
        }
        final Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Return heading, roll, pitch
        return new OrientationAngles(angles.firstAngle, angles.secondAngle, angles.thirdAngle);
    }

    /**
     * Redundant method, returns Acceleration object with xAccel, yAccel, zAccell components.
     * @param imu   Reference to IMU
     * @return Acceleration object
     */
    static public Acceleration getAccelerationComponents(BNO055IMU imu) {
        // If IMU is missing, do nothing.
        if (imu == null) {
            return new Acceleration(DistanceUnit.METER,-999, -999, -999,1000);
        }
        return  imu.getLinearAcceleration();
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    static protected String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static protected String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    static protected String formatAcceleration(double accel) {
        return String.format(Locale.getDefault(), "%.3f", accel);
    }

    // Calculation helpers
    // Initial and final headings are just used to help comparisons of orientation.
    private double headingCompensation = 0;
    private double initialHeading = 0;
    private double finalHeading = 0;

   public void setCompensatedHeading(double compensatedHeadingDegrees) {
       headingCompensation = compensatedHeadingDegrees - heading;
   }

   public double getCompensatedHeading() {
       return heading + headingCompensation;
   }

   public void setInitialHeading() {
       initialHeading = heading;
   }

   public void setFinalHeading() {
       finalHeading = heading;
   }

   public double getHeadingChange() {
       return finalHeading - initialHeading;
   }
}
