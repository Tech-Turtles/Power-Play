package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.opencv.core.Rect;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.*;


@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double driveSpeed = 1.0;
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double linearSpeed = 1.0;
    public static double lateralSpeed = 1.0;
    public static double rotationSpeed = 1.0;
    public static int direction = 1;

    private final Executive.StateMachine<Manual> stateMachine;
    private TrajectoryRR trajectoryRR;

    enum DriveMode {
        NORMAL_ROBOT_CENTRIC,
        NORMAL_FIELD_CENTRIC
    }

    private DriveMode currentDriveMode = DriveMode.NORMAL_ROBOT_CENTRIC;

    public Manual() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        stateMachine.changeState(DRIVE, new Drive_Manual());
        stateMachine.changeState(TURRET, new Turret_Stop());
        stateMachine.init();

        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
        trajectoryRR = new TrajectoryRR(this.mecanumDrive);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
//        mecanumDrive.setPoseEstimate(new Pose2d(-72+8.5,0));
        mecanumDrive.setPoseEstimate(new Pose2d());
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
        mecanumDrive.update(packet);
        displayTelemetry();
    }

    class Drive_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
//            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
//            Pose2d driveDirection = new Pose2d();
//
//            if(opMode.primary.AOnce()) {
//                precisionMode = precisionMode == 1 ? precisionPercentage : 1;
//            }
//
//            if(primary.YOnce()) {
//                direction = -direction;
//            }
//
//            if(primary.startOnce())
//                currentDriveMode = currentDriveMode == DriveMode.NORMAL_ROBOT_CENTRIC ? DriveMode.NORMAL_FIELD_CENTRIC : DriveMode.NORMAL_ROBOT_CENTRIC;
//
//            switch (currentDriveMode) {
//                case NORMAL_ROBOT_CENTRIC:
//                    driveDirection = new Pose2d(
//                            (-primary.left_stick_y * direction) * linearSpeed * precisionMode,
//                            (-primary.left_stick_x * direction) * lateralSpeed * precisionMode,
//                            -primary.right_stick_x * rotationSpeed * precisionMode
//                    );
//                    break;
//                case NORMAL_FIELD_CENTRIC:
//                    Vector2d fieldFrameInput = new Vector2d(
//                            -gamepad1.left_stick_y * linearSpeed * precisionMode,
//                            -gamepad1.left_stick_x * lateralSpeed * precisionMode);
//
//                    Vector2d robotFrameInput = fieldFrameInput
//                            .rotated(-poseEstimate.getHeading() + Math.toRadians(90.0));
//                    driveDirection = new Pose2d(
//                            robotFrameInput.getX(), robotFrameInput.getY(),
//                            -primary.right_stick_x * rotationSpeed * precisionMode
//                    );
//                    break;
//            }
//
//            if(currentDriveMode != DriveMode.NORMAL_FIELD_CENTRIC && Math.abs(primary.right_stick_x) > 0.1) {
//                currentDriveMode = DriveMode.NORMAL_ROBOT_CENTRIC;
//            }
//
//            mecanumDrive.setWeightedDrivePower(driveDirection);

//            if(primary.X())
//                opMode.motorUtility.setPower(Motors.FRONT_RIGHT, 1.0 * precisionMode * direction);
//            else
//                opMode.motorUtility.setPower(Motors.FRONT_RIGHT, 0.0);

            if(primary.X() && !stateMachine.getCurrentStateByType(TURRET).equals(Turret_Cone_Align.class)) {
                stateMachine.changeState(TURRET, new Turret_Cone_Align());
            } else if(!primary.X() && !stateMachine.getCurrentStateByType(TURRET).equals(Turret_Stop.class)) {
                stateMachine.changeState(TURRET, new Turret_Stop());
            }
        }
    }
    //ToDo Implement distance estimation + driving & grabbing cone autonomously
    class Turret_Cone_Align extends Executive.StateBase<Manual> {

        @Override
        public void update() {
            super.update();
            double turn = calculateTurn();
            telemetry.addData("turn", turn);
            motorUtility.setPower(Motors.FRONT_RIGHT, turn * precisionMode * 0.5);
        }

        private double calculateTurn() {
            if(visionDetection == null)
                return 0.0;

            Rect r = visionDetection.getPipeline().getBiggestCone();

            if(!(r.area() > 20))
                return 0.0;

            double coneCenter = r.x + (r.width/2.0);
            // Return if the cone is within tolerances (cone center is within 40 px of camera center)
            if(coneCenter > 310.0 && coneCenter < 330.0)
                return 0.0;

            double direction = coneCenter < 300.0 ? 1.0 : -1.0;
            double turn = Math.abs(coneCenter - 320) / 320.0;
            return turn * direction;
        }
    }

    class Turret_Stop extends Executive.StateBase<Manual> {
        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            motorUtility.setPower(Motors.FRONT_RIGHT, 0.0);
        }
    }

    void displayTelemetry() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

        telemetry.addData("Precision mode:      ", df.format(precisionMode));
        telemetry.addData("X:                   ", df.format(poseEstimate.getX()));
        telemetry.addData("Y:                   ", df.format(poseEstimate.getY()));
        telemetry.addData("Heading:             ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        if(packet != null) {
            packet.put("Precision mode:      ", df.format(precisionMode));
            packet.put("Drive speed:         ", df.format(driveSpeed));
            packet.put("Precision speed:     ", df.format(precisionPercentage));
            packet.put("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
        }
    }

    void stopAutoDriving() {
        mecanumDrive.cancelFollowing();
        mecanumDrive.setDrivePower(new Pose2d());
    }
}
