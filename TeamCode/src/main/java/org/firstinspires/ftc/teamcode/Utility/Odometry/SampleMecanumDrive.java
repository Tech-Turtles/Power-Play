package org.firstinspires.ftc.teamcode.Utility.Odometry;

import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants.kV;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.0;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static boolean useIMU = false;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private final LinkedList<Pose2d> poseHistory;

    private final DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private final List<DcMotorEx> motors;
    private volatile BNO055IMU imu;

    private final VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    public SampleMecanumDrive(HardwareMap hardwareMap, RobotHardware opmode) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        if(opmode != null)
            dashboard = opmode.getDashboard();
        else {
            dashboard = FtcDashboard.getInstance();
            dashboard.setTelemetryTransmissionInterval(25);
        }

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(0.5)), 0.5);

        poseHistory = new LinkedList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        if(useIMU) {
            imu = hardwareMap.get(BNO055IMU.class, IMU.IMU1.getName());
            if (imu != null) {
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);

                // If your hub is mounted vertically, remap the IMU axes so that the z-axis points
                // upward (normal to the floor) using a command like the following:
                BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
            }
        }

        if(opmode != null) {
            leftFront = opmode.motorUtility.getMotorReference(Motors.FRONT_LEFT);
            leftRear = opmode.motorUtility.getMotorReference(Motors.BACK_LEFT);
            rightRear = opmode.motorUtility.getMotorReference(Motors.BACK_RIGHT);
            rightFront = opmode.motorUtility.getMotorReference(Motors.FRONT_RIGHT);
        } else {
            leftFront = hardwareMap.get(DcMotorEx.class, Motors.FRONT_LEFT.getConfigName());
            leftRear = hardwareMap.get(DcMotorEx.class, Motors.BACK_LEFT.getConfigName());
            rightRear = hardwareMap.get(DcMotorEx.class, Motors.BACK_RIGHT.getConfigName());
            rightFront = hardwareMap.get(DcMotorEx.class, Motors.FRONT_RIGHT.getConfigName());

            leftFront.setDirection(Motors.FRONT_LEFT.getDirection());
            leftRear.setDirection(Motors.BACK_LEFT.getDirection());
            rightRear.setDirection(Motors.BACK_RIGHT.getDirection());
            rightFront.setDirection(Motors.FRONT_RIGHT.getDirection());
        }

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // If desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
//        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        if(useIMU)
            setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));
        else
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void cancelFollowing() {
        mode = Mode.IDLE;
    }
	
    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update(TelemetryPacket packet) {
        updatePoseEstimate();


        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        Canvas fieldOverlay = null;
        if(packet != null) {
            fieldOverlay = packet.fieldOverlay();

            packet.put("mode", mode);

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", Math.toDegrees(currentPose.getHeading()));

            packet.put("xError", lastError.getX());
            packet.put("yError", lastError.getY());
            packet.put("headingError", Math.toDegrees(lastError.getHeading()));
        }

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                if(packet != null) {
                    fieldOverlay.setStroke("#4CAF50");
                    DashboardUtil.drawRobot(fieldOverlay, newPose);
                }

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                if(packet != null) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke("#4CAF50");
                    DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                    double t = follower.elapsedTime();
                    DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                    fieldOverlay.setStroke("#3F51B5");
                    DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                }

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        if(packet != null) {
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
        }
    }

    public void waitForIdle() {
        TelemetryPacket packet;
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            packet = new TelemetryPacket();
            update(packet);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public boolean isIdle() {
        return mode == Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        try {
            leftFront.setPower(v);
        } catch (NullPointerException ignore) {}
        try {
            leftRear.setPower(v1);
        } catch (NullPointerException ignore) {}
        try {
            rightRear.setPower(v2);
        } catch (NullPointerException ignore) {}
        try {
            rightFront.setPower(v3);
        } catch (NullPointerException ignore) {}
    }

    @Override
    public double getRawExternalHeading() {
        if(useIMU)
            return imu.getAngularOrientation().firstAngle;
        return 0;
    }

    @Nullable
    @Override
    public Double getExternalHeadingVelocity() {
        if(!useIMU)
            return null;
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    double getBatteryVoltage(HardwareMap hardwareMap) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
	
    public void clearEstimatedPose() {
        poseHistory.clear();
        setPoseEstimate(new Pose2d(0,0,0));
    }
}
