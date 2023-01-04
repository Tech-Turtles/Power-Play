package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.INTAKE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.SLIDE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.TURRET;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.deadzone;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.CombinedTracker;
import org.firstinspires.ftc.teamcode.Vision.TrackType;
import org.opencv.core.Rect;


@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {
    
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.2625;
    public static double clawGrab = CLAW_OPEN;
    public static double linearSpeed = 1.0;
    public static double lateralSpeed = 1.0;
    public static double rotationSpeed = 1.0;
    public static double turretSpeed = 0.8;
    public static double turretMultiplier = 0.8;

    public static double turretA = 1000;
    public static double turretV = 1500;

    double theta = Math.toRadians(0.0);

    public static Configuration.ServoPosition armPosition = Configuration.ServoPosition.HOLD;

    public static int contourIndex = 0;

    public static Pose2d startingPosition = new Pose2d();

    private final Executive.StateMachine<Manual> stateMachine;
    public static double liftSpeed = 0.7;

    private double prevArmPos = armPosition.getLeft();
    public static double armOffset = -0.15;

    enum DriveMode {
        NORMAL_ROBOT_CENTRIC,
        CONSTANT_HEADING,
        NORMAL_FIELD_CENTRIC
    }

    public static DriveMode currentDriveMode = DriveMode.NORMAL_FIELD_CENTRIC;

    public Manual() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        armPosition = Configuration.ServoPosition.HOLD;

        stateMachine.changeState(INTAKE, new Horizontal_Arm_Position(armPosition));
        stateMachine.init();

        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
    }

    @Override
    public void start() {
        super.start();
        mecanumDrive.setPoseEstimate(startingPosition);
        stateMachine.changeState(DRIVE, new Drive_Manual());
        stateMachine.changeState(TURRET, new Turret_Manual());
        stateMachine.changeState(SLIDE, new Slide_Manual());
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
            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            if(primary.AOnce())
                precisionMode = precisionMode == 1 ? precisionPercentage : 1;

            if(primary.YOnce())
                mecanumDrive.setPoseEstimate(new Pose2d(mecanumDrive.getPoseEstimate().vec(), 0.0));

            if(primary.BOnce())
                currentDriveMode = currentDriveMode == DriveMode.NORMAL_FIELD_CENTRIC ? DriveMode.NORMAL_ROBOT_CENTRIC : DriveMode.NORMAL_FIELD_CENTRIC;

            switch (currentDriveMode) {
                case NORMAL_ROBOT_CENTRIC:
                    driveDirection = new Pose2d(
                            (-primary.left_stick_y) * linearSpeed * precisionMode,
                            (-primary.left_stick_x) * lateralSpeed * precisionMode,
                            -primary.right_stick_x * rotationSpeed * precisionMode
                    );
                    break;
                case NORMAL_FIELD_CENTRIC:
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y * linearSpeed * precisionMode,
                            -gamepad1.left_stick_x * lateralSpeed * precisionMode);

                    Vector2d robotFrameInput = fieldFrameInput
                            .rotated(-poseEstimate.getHeading())
                            .rotated(Math.toRadians(90.0));
                    driveDirection = new Pose2d(
                            robotFrameInput.getX(), robotFrameInput.getY(),
                            -primary.right_stick_x * rotationSpeed * precisionMode
                    );
                    theta = mecanumDrive.getPoseEstimate().getHeading();
                    break;
            }

            mecanumDrive.setWeightedDrivePower(driveDirection);

            if(secondary.Y() && !stateMachine.getCurrentStateByType(TURRET).equals(Turret_Cone_Align_Bang.class)) {
                stateMachine.changeState(TURRET, new Turret_Cone_Align_Bang());
            } else if(!stateMachine.getCurrentStateByType(TURRET).equals(Turret_Manual.class) && !secondary.Y() && !stateMachine.getCurrentStateByType(TURRET).equals(Turret_Angle.class)) {
                stateMachine.changeState(TURRET, new Turret_Manual());
            }

            if(secondary.rightTriggerOnce())
                clawGrab = clawGrab == CLAW_CLOSED ? CLAW_OPEN : CLAW_CLOSED;

            servoUtility.setAngle(Servos.CLAW, clawGrab);

            if(secondary.AOnce()) {
                stateMachine.changeState(SLIDE, new Slide_Position(Configuration.LOW_POS));
            } else if(secondary.XOnce()) {
                stateMachine.changeState(SLIDE, new Slide_Position(Configuration.HIGH_POS));
            }

            if(secondary.BOnce()) {
                CombinedTracker.trackType = CombinedTracker.trackType.equals(TrackType.CONE) ? TrackType.POLE : TrackType.CONE;
            }

            if(secondary.left_trigger > deadzone) {
                if(visionDetection == null)
                    return;
                CombinedTracker l = visionDetection.getLeftPipeline(), r = visionDetection.getRightPipeline();
                if(l == null || r == null)
                    return;
                // alpha
                double triangleRightAngle = 90 - r.getCameraAngle() + r.getPoleAngle();
                // beta
                double triangleLeftAngle = 90 + l.getCameraAngle() - l.getPoleAngle();

                double gamma = 180 - triangleLeftAngle - triangleRightAngle,
//                rightCameraDist = Math.sin(Math.toRadians(triangleLeftAngle)) * ((Configuration.CAMERA_DISTANCE_IN)/(Math.toRadians(gamma))),
                        leftCameraDist = Math.sin(Math.toRadians(triangleRightAngle)) *
                                ((Configuration.CAMERA_DISTANCE_IN)/Math.sin(Math.toRadians(gamma))),
                        frontDist = Math.abs(leftCameraDist * Math.sin(Math.toRadians(triangleLeftAngle)));

                double pos = Double.parseDouble(RobotHardware.df.format((Math.toDegrees(Math.acos((frontDist - 3.0)/14.0)) + 90.0) / 180.0));
                pos = CombinedTracker.trackType.equals(TrackType.CONE) ? pos + Configuration.ARM_CONE_OFFSET_IN : pos;

                try {
                    servoUtility.setAngle(Servos.LEFT_ARM, Range.clip((pos + armOffset), 0.0, 1.0));
                    servoUtility.setAngle(Servos.RIGHT_ARM, Range.clip(pos + armOffset, 0.0, 1.0));
                    prevArmPos = Range.clip(pos + armOffset, 0.0, 1.0);
                } catch(IllegalArgumentException ignore) {
                    servoUtility.setAngle(Servos.LEFT_ARM, prevArmPos);
                    servoUtility.setAngle(Servos.RIGHT_ARM, prevArmPos);
                }

                armPosition = Configuration.ServoPosition.NONE;

                stateMachine.changeState(INTAKE, new Horizontal_Arm_Position(prevArmPos, prevArmPos));
            } else {
                if (secondary.rightBumperOnce() && !armPosition.equals(Configuration.ServoPosition.INTAKE)) {
                    switch (armPosition) {
                        case START:
                        case NONE:
                            armPosition = Configuration.ServoPosition.HOLD;
                            break;
                        case HOLD:
                            armPosition = Configuration.ServoPosition.INTERMEDIARY;
                            break;
                        case INTERMEDIARY:
                            armPosition = Configuration.ServoPosition.INTAKE;
                    }
                    stateMachine.changeState(INTAKE, new Horizontal_Arm_Position(armPosition));
                } else if (secondary.leftBumperOnce() && !armPosition.equals(Configuration.ServoPosition.HOLD)) {
                    switch (armPosition) {
                        case NONE:
                        case INTERMEDIARY:
                            armPosition = Configuration.ServoPosition.HOLD;
                            break;
                        case INTAKE:
                            armPosition = Configuration.ServoPosition.INTERMEDIARY;
                    }
                    stateMachine.changeState(INTAKE, new Horizontal_Arm_Position(armPosition));
                }
            }

            double distance = getDistance(RevDistanceSensor.CLAW_DISTANCE);
            telemetry.addData("Distance", distance);
            if(distance < Configuration.CLAW_DISTANCE_IN && CombinedTracker.trackType.equals(TrackType.CONE) && armPosition.equals(Configuration.ServoPosition.INTAKE)) {
                clawGrab = CLAW_CLOSED;
            }

            if(secondary.dpadUpOnce()) {
                stateMachine.changeState(TURRET, new Turret_Angle(0.0));
            } else if(secondary.dpadRightOnce()) {
                stateMachine.changeState(TURRET, new Turret_Angle(-90.0));
            } else if(secondary.dpadDownOnce()) {
                stateMachine.changeState(TURRET, new Turret_Angle(180.0));
            } else if(secondary.dpadLeftOnce()) {
                stateMachine.changeState(TURRET, new Turret_Angle(90.0));
            }

            if(secondary.leftTriggerOnce()) {
                contourIndex++;
            } else if(secondary.rightStickButtonOnce()) {
                contourIndex = 0;
            }
        }
    }

    class Slide_Position extends Executive.StateBase<Manual> {
        private final double setpoint;
        private PIDFController controllerLeft, controllerRight;
        private MotionProfile activeProfile;
        private double profileStart;

        Slide_Position(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            controllerLeft = opMode.motorUtility.getController(Motors.SLIDE_LEFT);
            controllerRight = opMode.motorUtility.getController(Motors.SLIDE_RIGHT);
            profileStart = stateTimer.seconds();
            MotionState start = new MotionState(opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT), 0, 0, 0);
            MotionState goal = new MotionState(setpoint, 0, 0, 0);
            activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1600, 800);
        }

        @Override
        public void update() {
            super.update();

            if(Math.abs(secondary.left_stick_y) > 0.2) {
                stateMachine.changeState(SLIDE, new Slide_Manual());
                profileStart = 0;
                activeProfile = null;
                return;
            }

            double profileTime = stateTimer.seconds() - profileStart;

            MotionState motionState = activeProfile.get(profileTime);

            controllerLeft.setTargetPosition(motionState.getX());
            controllerLeft.setTargetVelocity(motionState.getV());
            controllerLeft.setTargetAcceleration(motionState.getA());

            controllerRight.setTargetPosition(motionState.getX());
            controllerRight.setTargetVelocity(motionState.getV());
            controllerRight.setTargetAcceleration(motionState.getA());


            opMode.motorUtility.setPower(Motors.SLIDE_RIGHT, controllerRight.update(opMode.motorUtility.getEncoderValue(Motors.SLIDE_RIGHT), opMode.motorUtility.getVelocity(Motors.SLIDE_RIGHT)));
            opMode.motorUtility.setPower(Motors.SLIDE_LEFT, controllerLeft.update(opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT), opMode.motorUtility.getVelocity(Motors.SLIDE_LEFT)));

            isDone = (stateTimer.seconds() - profileStart) > activeProfile.duration();
        }
    }

    class Slide_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(Math.abs(secondary.left_stick_y) < 0.2) {
                stateMachine.changeState(SLIDE, new Slide_Position(motorUtility.getEncoderValue(Motors.SLIDE_LEFT)));
                return;
            }

            if(-secondary.left_stick_y > 0.2 && opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT) <= 100)
                stateMachine.changeState(SLIDE, new Slide_Position(motorUtility.getEncoderValue(Motors.SLIDE_LEFT)));

            opMode.motorUtility.setPower(Motors.SLIDE_RIGHT, -secondary.left_stick_y < 0 ? -secondary.left_stick_y * liftSpeed : -secondary.left_stick_y);
            opMode.motorUtility.setPower(Motors.SLIDE_LEFT, -secondary.left_stick_y < 0 ? -secondary.left_stick_y * liftSpeed : -secondary.left_stick_y);
        }
    }

    class Horizontal_Arm_Position extends Executive.StateBase<Manual> {
        private final double l, r;
        Horizontal_Arm_Position(Configuration.ServoPosition servoPosition) {
            l = servoPosition.getLeft();
            r = servoPosition.getRight();
        }

        Horizontal_Arm_Position(double l, double r) {
            this.l = l;
            this.r = r;
        }

        @Override
        public void update() {
            super.update();
            servoUtility.setAngle(Servos.LEFT_ARM, l);
            servoUtility.setAngle(Servos.RIGHT_ARM, r);
        }
    }

    //ToDo Implement distance estimation + driving & grabbing cone autonomously
    class Turret_Cone_Align extends Executive.StateBase<Manual> {

        @Override
        public void update() {
            super.update();
            PIDFController controller = motorUtility.getController(Motors.TURRET);
            if(controller == null)
                return;

            //ToDo See if a motion profile is necessary
            double turn = calculateTurn() * turretMultiplier;
            
            controller.setTargetPosition(motorUtility.getEncoderValue(Motors.TURRET) + (turn * Configuration.TURRET_TICKS_PER_DEGREE));
            controller.setTargetAcceleration(turretA);
            controller.setTargetVelocity(turretV);

            motorUtility.setPower(Motors.TURRET, controller.update(motorUtility.getEncoderValue(Motors.TURRET), motorUtility.getVelocity(Motors.TURRET)));

            telemetry.addData("Turret Turn", turn);
        }

        private double calculateTurn() {
            if(visionDetection == null)
                return 0.0;

            visionDetection.getLeftPipeline().setContourIndex(contourIndex);

            Rect r = visionDetection.getLeftPipeline().getBiggestCone();

            if(!(r.area() > 10))
                return 0.0;

            double coneCenter = r.x + (r.width/2.0);

            if(coneCenter > 312.0 && coneCenter < 328.0)
                return 0.0;

            double direction = coneCenter < 320.0 ? 1.0 : -1.0;
            double turnDegrees = Math.abs(coneCenter - 320) / 9.0;
            return turnDegrees * direction;
        }

        private double wrapAngle(double angle) {
            return angle > 180 ? -(angle - 180) : (angle < -180 ? -(angle + 180) : angle);
        }
    }

    class Turret_Cone_Align_Bang extends Executive.StateBase<Manual> {

        @Override
        public void update() {
            super.update();
            motorUtility.setPower(Motors.TURRET, calculateTurn() * precisionMode);
        }

        private double calculateTurn() {
            if(visionDetection == null)
                return 0.0;


            Rect r = opMode.visionDetection.getRightPipeline().getBiggestCone();
            Rect l = opMode.visionDetection.getLeftPipeline().getBiggestCone();

            if(!(r.area() > 80))
                return 0.0;

            double coneCenterL = l.x + (l.width/2.0), coneCenterR = r.x + (r.width/2.0);
            double direction = coneCenterL < 320 - coneCenterR ? 1.0 : -1.0;
            double turn = Math.abs(coneCenterL - (320 - coneCenterR)) / 200.0;

            if(Math.abs(coneCenterL - (320 - coneCenterR)) < 5)
                return 0.0;

            return Range.clip(turn * turretMultiplier, 0.0, 1.0) * direction;
        }
    }

    class Turret_Angle extends Executive.StateBase<Manual> {
        private double angle;

        Turret_Angle(double angle) {
            this.angle = angle;
        }

        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            angle = angle * Configuration.TURRET_TICKS_PER_DEGREE;
        }

        @Override
        public void update() {
            super.update();
            if(Math.abs(secondary.right_stick_x) > 0.3)
                stateMachine.changeState(TURRET, new Turret_Manual());

            isDone = motorUtility.goToPosition(Motors.TURRET, (int) angle, 1.0);
        }
    }

    class Turret_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            motorUtility.setPower(Motors.TURRET, -secondary.right_stick_x * turretSpeed);
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
            packet.put("Precision speed:     ", df.format(precisionPercentage));
            packet.put("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
        }
    }

    void stopAutoDriving() {
        mecanumDrive.cancelFollowing();
        mecanumDrive.setDrivePower(new Pose2d());
    }
}
