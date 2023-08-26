package org.firstinspires.ftc.teamcode.Utility;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareTypes.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.ExpansionHubs;
import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.HardwareTypes.MotorTypes;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.HardwareTypes.Webcam;
import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.Utility.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.PIDFConfig;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.SwerveControllerConfiguration;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.SwerveDriveConfiguration;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.SwerveModuleConfiguration;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.SwerveModulePhysicalCharacteristics;
import org.firstinspires.ftc.teamcode.Utility.Swerve.hardware.SwerveAbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Vision.CombinedDetector;

import java.text.DecimalFormat;
import java.util.HashMap;

/**
 * @author Christian E, Ashley T
 * Revamped RobotHardware, better readability, new methods, more organized, easier to build upon.
 * <p>
 * A class made to act as a foundation for all other OpModes, responsible for retrieving hardware
 * based on the configuration-names registered within the enums in the HardwareTypes package.
 * Contains wrapper functions to prevent errors when a device is not detected.
 * Handles LynxModule cache clearing.
 * Handles gamepad updates.
 * Handles Dashboard initialization & sending telemetry packet.
 * Stops all continuous motion devices on {@link #stop()}.
 */
public class RobotHardware extends OpMode {
    // Hashmaps to store the hardware object with the key being the enum values.
    private final HashMap<Motors, DcMotorEx> motors = new HashMap<>();
    private final HashMap<Servos, ServoImplEx> servos = new HashMap<>();
    private final HashMap<ContinuousServo, CRServoImplEx> crServos = new HashMap<>();
    private final HashMap<ColorSensor, RevColorSensorV3> colorSensors = new HashMap<>();
    private final HashMap<RevDistanceSensor, DistanceSensor> distanceSensors = new HashMap<>();
    // Utility objects to access hardware methods.
    public final MotorUtility motorUtility = new MotorUtility();
    public final ServoUtility servoUtility = new ServoUtility();
    // Decimal format objects for easy string formatting.
    public static final DecimalFormat df = new DecimalFormat("0.00"), df_precise = new DecimalFormat("0.0000");
    // Hub & hub sensor objects.
    public VoltageSensor batteryVoltageSensor;
    protected LynxModule expansionHub1, expansionHub2;
    // Timer to keep track of the period & period mean times.
    public final ElapsedTimer period = new ElapsedTimer();
    // Controller objects that act as a more intuitive wrapper for the FTC gamepad class.
    public Controller primary, secondary;

    private SwerveDrive drive;

    private BNO055IMUImpl imu;

    public static double totalTime = 0.0;

    public CombinedDetector visionDetection;
    public SampleMecanumDrive mecanumDrive;
    // Acme dashboard objects.
    private FtcDashboard dashboard;
    public TelemetryPacket packet;

    public static Pose2d lastPosition = new Pose2d(0,0,0);

    public class MotorUtility {

        private DcMotorEx m;

        private void getMotor(Motors motor) {
            m = motors.get(motor);
            if (m == null && packet != null)
                packet.addLine("Motor Missing: " + motor.name());
        }

        public DcMotorEx getMotorReference(Motors motor) {
            DcMotorEx m = motors.get(motor);
            if (m == null && packet != null)
                packet.addLine("Motor Missing: " + motor.name());
            return m;
        }

        public double getPower(Motors motor) {
            getMotor(motor);
            if (m == null) return 0;
            return m.getPower();
        }

        public void setPower(Motors motor, double power) {
            getMotor(motor);
            if (m != null)
                m.setPower(power);
        }

        public int getEncoderValue(Motors motor) {
            getMotor(motor);
            if (m == null) return -1;
            return m.getCurrentPosition();
        }

        public double getVelocity(Motors motor) {
            getMotor(motor);
            if (m == null) return -1;
            return m.getVelocity();
        }

        public double getCurrent(Motors motor) {
            getMotor(motor);
            if (m == null) return -1;
            return m.getCurrent(CurrentUnit.AMPS);
        }

        public void stopAllMotors() {
            for (Motors motor : Motors.values())
                setPower(motor, 0);
        }

        public void stopResetAllMotors() {
            motors.forEach((k, v) -> v.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        }

        public void stopResetMotor(Motors motor) {
            getMotor(motor);
            if (m != null)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void setPIDFCoefficients(DcMotor.RunMode runmode, PIDFCoefficients pidfCoefficients, Motors... motors) {
            for(Motors motor : motors) {
                getMotor(motor);
                if(m == null) continue;
                m.setPIDFCoefficients(runmode, pidfCoefficients);
            }
        }

        public void setPIDFCoefficientsCompensated(Motors motor, DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
            PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d,
                    coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            );
            setPIDFCoefficients(runMode, compensatedCoefficients, motor);
        }

        public void setTypeMotorsRunMode(MotorTypes type, DcMotor.RunMode runMode) {
            for (Motors motor : Motors.values()) {
                if (!motor.getType().equals(type)) continue;
                getMotor(motor);
                if (m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setMotorsRunMode(DcMotor.RunMode runMode, Motors... motors) {
            for (Motors motor : motors) {
                getMotor(motor);
                if (m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setTypeMotorsZeroPowerBehavior(MotorTypes type, DcMotor.ZeroPowerBehavior behavior) {
            for (Motors motor : Motors.values()) {
                if (!motor.getType().equals(type)) continue;
                getMotor(motor);
                if (m == null) continue;
                m.setZeroPowerBehavior(behavior);
            }
        }

        public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior, Motors... motors) {
            for (Motors motor : motors) {
                getMotor(motor);
                if (m == null) continue;
                m.setZeroPowerBehavior(behavior);
            }
        }

        private void initializeMotors() {
            stopResetAllMotors();
            motors.forEach((k, v) -> {
                v.setMode(k.getRunMode());
                v.setZeroPowerBehavior(k.getZeroPowerBehavior());
                v.setDirection(k.getDirection());
                if(k.getPidCoefficients() != null)
                    k.setController(new PIDFController(k.getPidCoefficients()));
            });
        }

        /**
         * Sets the drive chain power.
         *
         * @param left  The power for the left two motors.
         * @param right The power for the right two motors.
         */
        public void setDriveForTank(double left, double right) {
            setPower(Motors.FRONT_LEFT, left);
            setPower(Motors.BACK_LEFT, left);
            setPower(Motors.FRONT_RIGHT, right);
            setPower(Motors.BACK_RIGHT, right);
        }

//        /**
//         * @param motor The motor that will be driven
//         * @param targetTicks The position where the motor will be driven. Must be in encoder Ticks
//         * @param power The power at which the robot will be driven
//         * @param rampThreshold The position when the robot will start slowing the motor down before its destination
//         * @return Returns whether or not the motor arrived to the specified position
//         */
//        public boolean goToPosition(Motors motor, int targetTicks, double power, double rampThreshold) {
//            power = Range.clip(Math.abs(power), 0, 1);
//            int poweredDistance = 0;
//            int arrivedDistance = 50;
//            double maxRampPower = 1.0;
//            double minRampPower = 0.0;
//            int errorSignal = getEncoderValue(motor) - targetTicks;
//            double direction = errorSignal > 0 ? -1.0 : 1.0;
//            double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, maxRampPower, minRampPower);
//
//            if (Math.abs(errorSignal) >= poweredDistance) {
//                setPower(motor, direction * power * rampDownRatio);
//            } else {
//                setPower(motor, 0);
//            }
//
//            return Math.abs(errorSignal) <= arrivedDistance;
//        }

        //ToDo Prevent PID Controller from possibly being null
        public PIDFController getController(Motors motor) {
            getMotor(motor);
            if(motor == null) return null;
            PIDFController controller = motor.getController();
            if(controller == null && packet != null)
                packet.put("PID Controller Missing", motor.name());
            return controller;
        }

//        /**
//         * @param motor The motor that will be driven
//         * @param targetTicks The position where the motor will be driven. Must be in encoder Ticks
//         * @param power The power at which the robot will be driven
//         * @return Returns whether or not the motor arrived to the specified position
//         */
//        public boolean goToPosition(Motors motor, int targetTicks, double power) {
//            int rampDistanceTicks = 400;
//            return goToPosition(motor, targetTicks, power, rampDistanceTicks);
//        }
    }

    public class ServoUtility {
        ServoImplEx s;
        CRServoImplEx cr;

        private ServoImplEx getServo(Servos servo) {
            s = servos.get(servo);
            if (s == null && packet != null)
                packet.addLine("Servo Missing: " + servo.name());
            return s;
        }

        private CRServoImplEx getContinuousServo(ContinuousServo servo) {
            cr = crServos.get(servo);
            if (cr == null && packet != null)
                packet.addLine("CRServo Missing: " + servo.name());
            return cr;
        }

        public void setAngle(Servos servo, double pos) {
            s = getServo(servo);
            if (s != null)
                s.setPosition(pos);
        }

        public double getAngle(Servos servo) {
            s = getServo(servo);
            if (s != null)
                return s.getPosition();
            return -1;
        }

        public boolean moveServoAtRate(Servos servo, double targetPos, double rate) {
            boolean isMovementDone = false;
            double distanceThreshold = 0.05;
            rate = Range.clip(rate, 0, 10);
            targetPos = Range.clip(targetPos, 0, 1);
            double currentPosition = getAngle(servo);
            double distance = targetPos - currentPosition;
            double direction = targetPos > currentPosition ? 1 : -1;
            double nextPosition;
            if (Math.abs(distance) > distanceThreshold) {
                nextPosition = rate * direction * period.getLastPeriodSec() + currentPosition;
            } else {
                nextPosition = targetPos;
                isMovementDone = true;
            }
            nextPosition = Range.clip(nextPosition, 0, 1);
            setAngle(servo, nextPosition);

            return isMovementDone;
        }

        public void setPower(ContinuousServo continuousServo, double power) {
            cr = getContinuousServo(continuousServo);
            if(cr == null) return;
            cr.setPower(power);
        }

        /**
         * Scales the available movement range of the servo to be a subset of its maximum range. Subsequent
         * positioning calls will operate within that subset range. This is useful if your servo has
         * only a limited useful range of movement due to the physical hardware that it is manipulating
         * (as is often the case) but you don't want to have to manually scale and adjust the input
         * @param servo The servo being scaled.
         * @param min Minimum of the range.
         * @param max Max of the range.
         */
        public void setScale(Servos servo, double min, double max) {
            s = getServo(servo);
            if(s == null) return;
            s.scaleRange(min, max);
        }
    }

    public BNO055IMUImpl getIMUInstance() {
        if(imu == null) {
            imu = hardwareMap.get(BNO055IMUImpl.class, IMU.IMU1.getName());
            if(imu != null) {
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);

                // If your hub is mounted vertically, remap the IMU axes so that the z-axis points
                // upward (normal to the floor) using a command like the following:
                BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
            }
        }
        return imu;
    }

    public RevColorSensorV3 getColorSensor(ColorSensor colorSensor) {
        RevColorSensorV3 revColorSensorV3 = colorSensors.get(colorSensor);
        if(revColorSensorV3 == null && packet != null)
            packet.addLine("Sensor Missing: " + colorSensor.name());
        return revColorSensorV3;
    }

    private DistanceSensor getDistanceSensor(RevDistanceSensor revDistanceSensor) {
        DistanceSensor distanceSensor = distanceSensors.get(revDistanceSensor);
        if(distanceSensor == null && packet != null)
            packet.addLine("Sensor Missing: " + revDistanceSensor.name());
        return distanceSensor;
    }

    public double getDistance(RevDistanceSensor revDistanceSensor) {
        DistanceSensor distanceSensor = getDistanceSensor(revDistanceSensor);
        if(distanceSensor == null) return -1.0;
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double getDistance(RevColorSensorV3 colorSensor) {
        if(colorSensor == null) return -1.0;
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH);
    }

    public void loadVision() {
        visionDetection = new CombinedDetector(hardwareMap, Webcam.VISION_RIGHT.getName(), Webcam.VISION_LEFT.getName());
        visionDetection.init();
    }

    public void clearHubCache() {
        try {
            expansionHub1.clearBulkCache();
        } catch (Exception e) {
            if(packet != null)
                packet.addLine("Error: " + e.getMessage());
        }
        try {
            expansionHub2.clearBulkCache();
        } catch (Exception e) {
            if(packet != null)
                packet.addLine("Error: " + e.getMessage());
        }
    }

    public double getTime() {
        return time;
    }

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    public static String setFontColor(String color, String text) {
        return "<font color=\""+color+"\">"+text+"</font>";
    }

    public static String setHeader(int headerNumber, String text) {
        return "<h"+headerNumber+">"+text+"</h"+headerNumber+">";
    }

    SwerveControllerConfiguration swerveControllerConfiguration;
    SwerveAbsoluteEncoder leftFront;
    @Override
    public void init() {
        totalTime = getTime();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        packet = new TelemetryPacket();

        try {
            expansionHub1 = hardwareMap.get(LynxModule.class, ExpansionHubs.HUB1.getHub());
            expansionHub2 = hardwareMap.get(LynxModule.class, ExpansionHubs.HUB2.getHub());

            expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (IllegalArgumentException | NullPointerException e) {
            if(packet != null)
                packet.addLine("Error: " + e.getMessage());
            Log.wtf("RobotHardware: ", e.getMessage());
        }
        clearHubCache();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (Motors m : Motors.values()) {
            try {
                motors.put(m, hardwareMap.get(DcMotorEx.class, m.getConfigName()));
            } catch (IllegalArgumentException ignore) {}
        }
        new MotorUtility().initializeMotors();

        for (Servos s : Servos.values()) {
            try {
                ServoImplEx servo = hardwareMap.get(ServoImplEx.class, s.getConfigName());
                servos.put(s, servo);
                servo.setDirection(s.getDirection());
                servo.setPwmRange(new PwmControl.PwmRange(510, 2490));
            } catch (IllegalArgumentException ignore) {}
        }

        for (ContinuousServo c : ContinuousServo.values()) {
            try {
                CRServoImplEx crServo = hardwareMap.get(CRServoImplEx.class, c.getConfigName());
                crServos.put(c, crServo);
                crServo.setDirection(c.getDirection());
            } catch (IllegalArgumentException ignore) {}
        }

        for (ColorSensor c : ColorSensor.values()) {
            try {
                RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, c.getConfig());
                colorSensors.put(c, colorSensor);
                colorSensor.enableLed(false);
            } catch (IllegalArgumentException ignore) {}
        }

        for (RevDistanceSensor d : RevDistanceSensor.values()) {
            try {
                DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, d.getConfigName());
                distanceSensors.put(d, distanceSensor);
            } catch (IllegalArgumentException ignore) {}
        }

        SwerveModulePhysicalCharacteristics physicalCharacteristics = new SwerveModulePhysicalCharacteristics(
                8.181818, 1, 0.072, 1.1,
                12.0, 20, 20, 0.25,
                0.25, 1, 1, 0.0
        );
        leftFront = new SwerveAbsoluteEncoder(hardwareMap.get(AnalogInput.class, "leftFrontEncoder"));
        SwerveModuleConfiguration front_left = new SwerveModuleConfiguration(motorUtility.getMotorReference(Motors.FRONT_LEFT),
                servoUtility.getContinuousServo(ContinuousServo.FRONT_LEFT), leftFront,
                -18.0, 0.1778, 0.1778, new PIDFConfig(0.08,0.0),
                new PIDFConfig(0.08, 0.0), 2.0, physicalCharacteristics, "FrontLeft"
        );

        SwerveModuleConfiguration front_right = new SwerveModuleConfiguration(motorUtility.getMotorReference(Motors.FRONT_RIGHT),
                servoUtility.getContinuousServo(ContinuousServo.FRONT_RIGHT), new SwerveAbsoluteEncoder(hardwareMap.get(AnalogInput.class, "rightFrontEncoder")),
                0.0, -0.1778, 0.1778, new PIDFConfig(0.08,0.0),
                new PIDFConfig(0.08, 0.0), 2.0, physicalCharacteristics, "FrontRight"
        );

        SwerveModuleConfiguration back_left = new SwerveModuleConfiguration(motorUtility.getMotorReference(Motors.BACK_LEFT),
                servoUtility.getContinuousServo(ContinuousServo.BACK_LEFT), new SwerveAbsoluteEncoder(hardwareMap.get(AnalogInput.class, "leftBackEncoder")),
                0.0, 0.1778, -0.1778, new PIDFConfig(0.08,0.0),
                new PIDFConfig(0.08, 0.0), 2.0, physicalCharacteristics, "BackLeft"
        );

        SwerveModuleConfiguration back_right = new SwerveModuleConfiguration(motorUtility.getMotorReference(Motors.BACK_RIGHT),
                servoUtility.getContinuousServo(ContinuousServo.BACK_RIGHT), new SwerveAbsoluteEncoder(hardwareMap.get(AnalogInput.class, "rightBackEncoder")),
                0.0, -0.1778, -0.1778, new PIDFConfig(0.08,0.0),
                new PIDFConfig(0.08, 0.0), 2.0, physicalCharacteristics, "BackRight"
        );

        SwerveDriveConfiguration swerveDriveConfiguration = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[]{front_left, back_left, back_right, front_right}, IMU.IMU1, 2.0, false);

        swerveControllerConfiguration = new SwerveControllerConfiguration(swerveDriveConfiguration, new PIDFConfig(0.08, 0.0));
        drive = new SwerveDrive(swerveDriveConfiguration, swerveControllerConfiguration, this);

        PhotonCore.enable();

        primary = new Controller(gamepad1);
        secondary = new Controller(gamepad2);
        motorUtility.stopAllMotors();
        if(packet != null)
            dashboard.sendTelemetryPacket(packet);
        period.reset();
    }

    @Override
    public void init_loop() {
        totalTime = getTime();
        if(packet != null) {
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }
        clearHubCache();
        period.updatePeriodTime();
        primary.update();
        secondary.update();
    }

    @Override
    public void start() {
        if(packet != null) {
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }
        clearHubCache();
        motorUtility.stopAllMotors();
        period.reset();
    }

    @Override
    public void loop() {
        totalTime = getTime();
        if(packet != null) {
            dashboard.sendTelemetryPacket(packet);
//            packet = new TelemetryPacket();
            packet.clearLines();
        }
        clearHubCache();
        period.updatePeriodTime();

        primary.update();
        secondary.update();

        packet.put("Period Average: ", df_precise.format(period.getAveragePeriodSec()) + "s");
        packet.put("Absolute Encoder", leftFront.getCurrentPosition());

//        if(primary.X()) {
//            servoUtility.setPower(ContinuousServo.FRONT_LEFT, 1.0);
//        } else if(primary.Y()) {
//            servoUtility.setPower(ContinuousServo.FRONT_LEFT, -1.0);
//        } else {
//            servoUtility.setPower(ContinuousServo.FRONT_LEFT, 0.0);
//        }

        double xVelocity   = -primary.left_stick_y;
        double yVelocity   = primary.left_stick_x;
        double angVelocity = primary.right_stick_x;

        drive.drive(new Translation2d(xVelocity * swerveControllerConfiguration.maxSpeed,
                        yVelocity * swerveControllerConfiguration.maxSpeed),
                angVelocity * swerveControllerConfiguration.maxAngularVelocity,
                false, true, false);
    }
    
    /**
     * Stops all motors and calls requestOpModeStop() to end the opmode
     */
    @Override
    public void stop() {
        if(packet != null)
            dashboard.sendTelemetryPacket(packet);
        packet = null;
        dashboard = null;
        try {
            lastPosition = mecanumDrive.getPoseEstimate();
        } catch (Exception e) {
            Log.wtf("Unable to save positions", e.getMessage());
        }
        clearHubCache();
        motorUtility.stopAllMotors();
        for (ContinuousServo servo : ContinuousServo.values())
            servoUtility.setPower(servo, 0);

        //ToDo Add stop function to vision thread
    }
}