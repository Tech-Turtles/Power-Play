package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.INTAKE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.SLIDE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.TURRET;
import static org.firstinspires.ftc.teamcode.Utility.RobotHardware.df;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Opmodes.Driving.Manual;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.CombinedTracker;
import org.firstinspires.ftc.teamcode.Vision.TrackType;
import org.opencv.core.Rect;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode autoOpmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private TrajectoryRR trajectoryRR;
    private Signal signal = Signal.NONE;
    private final double visionTimeout = 2.0;
    public static boolean far = false;
    private int lastTurretPos = 0;
    private double lastArmPos = 0.7;
    public static double armOffset = -0.053;
    public static int maxIterations = 5;

    private final double HIGH_POLE_POS = 780.0, INTAKE_LOW_POS = Configuration.MEDIUM_POS - 60.0, CONE_STACK_TICKS = 45.0;

    public RobotStateContext(AutoOpmode autoOpmode, AllianceColor allianceColor, StartPosition startPosition) {
        this.autoOpmode = autoOpmode;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(autoOpmode);
        stateMachine.update();
    }

    @Override
    public void init() {
        trajectoryRR = new TrajectoryRR(autoOpmode.mecanumDrive);
        trajectoryRR.resetTrajectories(allianceColor);
        stateMachine.changeState(DRIVE, new Start());
        stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.HOLD, 0, Configuration.CLAW_CLOSED));
        stateMachine.init();
    }

    @Override
    public void update() {
        stateMachine.update();
        Pose2d poseEstimate = autoOpmode.mecanumDrive.getPoseEstimate();
        autoOpmode.telemetry.addData("Tag", autoOpmode.tagOfInterest == null ? "NONE" : autoOpmode.tagOfInterest.id);
        autoOpmode.telemetry.addData("X:        ", df.format(poseEstimate.getX()));
        autoOpmode.telemetry.addData("Y:        ", df.format(poseEstimate.getY()));
        autoOpmode.telemetry.addData("Heading:  ", df.format(Math.toDegrees(poseEstimate.getHeading())));
    }

    @Override
    public String getCurrentState() {
        return stateMachine.getCurrentStateByType();
    }

    /**
     * Start State
     * State that sets the robot's position to the start position.
     * Changes the routine based on start position.
     *
     * Trajectory: none
     * Next State: Initial
     */
    //ToDo Add support for far-start position autonomous
    class Start extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            if(far) {
                if(allianceColor.equals(AllianceColor.BLUE))
                    trajectoryRR.resetTrajectories(AllianceColor.RED);
                else
                    trajectoryRR.resetTrajectories(AllianceColor.BLUE);
            } else if(allianceColor.equals(AllianceColor.BLUE))
                trajectoryRR.resetTrajectories(AllianceColor.BLUE);

            switch (startPosition) {
                case AUDIENCE:
                    setupInitialPosition(trajectoryRR.getStartAudience());
                    nextState(DRIVE, new Scan());
                    break;
                case FAR:
                    setupInitialPosition(trajectoryRR.getStartFar());
                    nextState(DRIVE, new Scan());
                    break;
                default:
                   throw new IllegalArgumentException("Invalid start position");
            }
        }

        private void setupInitialPosition(Pose2d initialPosition) {
            opMode.mecanumDrive.setPoseEstimate(initialPosition);
        }
    }

    /**
     * Scan State
     * Get what april tag was identified during initialization
     *
     * Trajectory: none
     * Next State:
     */
    class Scan extends Executive.StateBase<AutoOpmode> {

        @Override
        public void update() {
            super.update();

            signal = Signal.getSignalByOrdinal(autoOpmode.tagOfInterest == null ? Configuration.DEFAULT_PARK_ORDINAL : autoOpmode.tagOfInterest.id);

            if(!signal.equals(Signal.NONE) || stateTimer.seconds() > visionTimeout) {
                CombinedTracker.coneColor = allianceColor.equals(AllianceColor.BLUE) ?
                        CombinedTracker.DETECT_COLOR.BLUE : CombinedTracker.DETECT_COLOR.RED;
                CombinedTracker.trackType = TrackType.NONE;
                nextState(DRIVE, new StartToHighPole());
            }
        }
    }

    class StartToSignalPark extends Executive.StateBase<AutoOpmode> {

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getSleeveTrajectory(allianceColor, signal));
        }

        @Override
        public void update() {
            super.update();

            if(opMode.mecanumDrive.isIdle())
                nextState(DRIVE, new Stop());
        }
    }

    class StartToHighPole extends Executive.StateBase<AutoOpmode> {

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryAudienceStartToHighPole());
            stateMachine.changeState(SLIDE, new SlidePosition(HIGH_POLE_POS));
            stateMachine.changeState(INTAKE, new ArmPosition(Configuration.ServoPosition.HOLD));
            stateMachine.changeState(TURRET, new TurretAngle(far ? (allianceColor.equals(AllianceColor.RED) ? (35.0) : (50.0)) : (allianceColor.equals(AllianceColor.RED) ? (50.0) : (35.0))));
            CombinedTracker.trackType = TrackType.POLE;
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            if(stateMachine.getStateReferenceByType(TURRET).isDone && opMode.visionDetection != null && !stateMachine.getCurrentStateByType(TURRET).equals(TurretVisionTrack.class))
                stateMachine.changeState(TURRET, new TurretVisionTrack());

            if(!isDone && (stateMachine.getStateReferenceByType(SLIDE).isDone || timer.seconds() > 2.0)) {
                isDone = true;
                timer.reset();
                stateMachine.changeState(INTAKE, new ArmPositionVisionDelayedIntake(Configuration.ServoPosition.PLACE, 0.5, Configuration.CLAW_OPEN));
            }

            if(timer.seconds() > 0.75 && isDone) {
                stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.HOLD, 0.0, Configuration.CLAW_OPEN));
                stateMachine.changeState(DRIVE, new HighPoleToStack(1));
            }
        }
    }

    class HighPoleToStack extends Executive.StateBase<AutoOpmode> {
        private final int iteration;
        private boolean hasArrived = false;
        HighPoleToStack(int iteration) {
            this.iteration = iteration;
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryAudienceHighPoleToStack());
            stateMachine.changeState(SLIDE, new SlidePosition(INTAKE_LOW_POS - (iteration * CONE_STACK_TICKS)));
            stateMachine.changeState(INTAKE, new IntakeDelayedArmPosition(Configuration.ServoPosition.LOW_INTAKE, 0.32, Configuration.CLAW_OPEN));
            stateMachine.changeState(TURRET, new TurretAngle(far ? (allianceColor.equals(AllianceColor.BLUE) ? (180.0) : (-90.0)) : (allianceColor.equals(AllianceColor.BLUE) ? (-90.0) : (180.0)), 0.3));
            CombinedTracker.trackType = TrackType.CONE;
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            if(!hasArrived) {
                opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryAudienceSlowStackGrab());
                hasArrived = true;
                return;
            }

            if(!isDone && (timer.seconds() > 0.45 || (opMode.getDistance(RevDistanceSensor.CLAW_DISTANCE) < 0.35 && opMode.getDistance(RevDistanceSensor.CLAW_DISTANCE) > 0.0))) {
                isDone = true;
                stateMachine.changeState(INTAKE, new IntakeDelayedArmPosition(Configuration.ServoPosition.LOW_INTAKE, 0.0, Configuration.CLAW_CLOSED));
                stateMachine.changeState(SLIDE, new SlidePosition(HIGH_POLE_POS));
                timer.reset();
                return;
            }

            if(isDone && timer.seconds() > 0.75) {
                stateMachine.changeState(DRIVE, new StackToHighPole(iteration + 1));
            }
        }
    }

    class StackToHighPole extends Executive.StateBase<AutoOpmode> {
        private final int iteration;
        StackToHighPole(int iteration) {
            this.iteration = iteration;
        }
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryAudienceStackToHighPole());
            stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.HOLD, 0, Configuration.CLAW_CLOSED));
            stateMachine.changeState(SLIDE, new SlidePosition(HIGH_POLE_POS));
            stateMachine.changeState(TURRET, new TurretAngle(lastTurretPos / Configuration.TURRET_TICKS_PER_DEGREE));
            if(iteration >= 4)
                CombinedTracker.trackType = TrackType.CONE;
            else
                CombinedTracker.trackType = TrackType.POLE;
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            if(stateMachine.getStateReferenceByType(TURRET).isDone && opMode.visionDetection != null && !stateMachine.getCurrentStateByType(TURRET).equals(TurretVisionTrack.class))
//                if(iteration >= 4)
//                    stateMachine.changeState(TURRET, new Executive.StateBase<AutoOpmode>() {
//                        @Override
//                        public void update() {
//                            super.update();
//                            isDone = opMode.motorUtility.goToPosition(Motors.TURRET, lastTurretPos, 1.0);
//                        }
//                    });
//                else
                    stateMachine.changeState(TURRET, new TurretVisionTrack());

            if(!isDone && (stateMachine.getStateReferenceByType(SLIDE).isDone || timer.seconds() > 2.0)) {
                isDone = true;
                timer.reset();
//                if(iteration >= 4)
//                    stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(lastArmPos, lastArmPos, 0.5, Configuration.CLAW_OPEN));
//                else
                    stateMachine.changeState(INTAKE, new ArmPositionVisionDelayedIntake(Configuration.ServoPosition.PLACE, 0.5, Configuration.CLAW_OPEN));
            }

//            if(!isDone && (stateMachine.getStateReferenceByType(SLIDE).isDone || timer.seconds() > 2.0) && (stateMachine.getStateReferenceByType(TURRET).isDone || timer.seconds() > 2.0)) {
//                isDone = true;
//                timer.reset();
//                stateMachine.changeState(SLIDE, new SlidePosition(675));
//                if(iteration == 4) {
//                    stateMachine.changeState(TURRET, new Executive.StateBase<AutoOpmode>() {
//                        @Override
//                        public void update() {
//                            super.update();
//                            isDone = opMode.motorUtility.goToPosition(Motors.TURRET, lastTurretPos, 1.0);
//                        }
//                    });
//                    stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(lastArmPos, lastArmPos, 1.0, Configuration.CLAW_OPEN));
//                } else if (opMode.visionDetection != null) {
//                    stateMachine.changeState(TURRET, new TurretVisionTrack());
//                    stateMachine.changeState(INTAKE, new ArmPositionVisionDelayedIntake(Configuration.ServoPosition.PLACE, 1.0, Configuration.CLAW_OPEN));
//                }
//            }

            if(timer.seconds() > 0.75 && isDone) {
                if(iteration >= maxIterations)
                    stateMachine.changeState(DRIVE, new HighPoleToPark());
                else
                    stateMachine.changeState(DRIVE, new HighPoleToStack(iteration));
            }
        }
    }

    class HighPoleToPark extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getHighPoleSleeveTrajectory(allianceColor, signal));
            stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.HOLD, 0, Configuration.CLAW_OPEN));
            stateMachine.changeState(SLIDE, new SlidePosition(Configuration.LOW_POS));
            stateMachine.changeState(TURRET, new TurretAngle(far ? (allianceColor.equals(AllianceColor.RED) ? (0.0) : (90.0)) : (allianceColor.equals(AllianceColor.RED) ? (90.0) : (0.0))));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            Manual.startingPosition = opMode.mecanumDrive.getPoseEstimate();

            stateMachine.changeState(SLIDE, new StopMotors(Motors.SLIDE_RIGHT, Motors.SLIDE_LEFT));
            stateMachine.changeState(TURRET, new StopMotors(Motors.TURRET));

            stateMachine.changeState(DRIVE, new Stop());
        }
    }

    static class ArmPosition  extends Executive.StateBase<AutoOpmode> {
        private final Configuration.ServoPosition servoPosition;
        ArmPosition(Configuration.ServoPosition servoPosition) {
            this.servoPosition = servoPosition;
        }

        @Override
        public void update() {
            super.update();
            opMode.servoUtility.setAngle(Servos.LEFT_ARM, servoPosition.getLeft());
            opMode.servoUtility.setAngle(Servos.RIGHT_ARM, servoPosition.getRight());
        }
    }

    static class ArmPositionAngle extends Executive.StateBase<AutoOpmode> {
        private final double l, r;
        ArmPositionAngle(double l, double r) {
            this.l = l;
            this.r = r;
        }

        @Override
        public void update() {
            super.update();
            if(timer.seconds() > 0.1) {
                timer.reset();
                opMode.servoUtility.setAngle(Servos.LEFT_ARM, l);
                opMode.servoUtility.setAngle(Servos.RIGHT_ARM, r);
            }
        }
    }

    class TurretVisionTrack extends Executive.StateBase<AutoOpmode> {
        PIDFController controller;
        private MotionProfile activeProfile;
        private double profileStart;

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
//            controller = opMode.motorUtility.getController(Motors.TURRET);
//            profileStart = timer.seconds();
//            MotionState start = new MotionState(opMode.motorUtility.getEncoderValue(Motors.TURRET), 0, 0, 0);
//            MotionState goal = new MotionState(opMode.motorUtility.getEncoderValue(Motors.TURRET) + calculateTurn() * Configuration.TURRET_TICKS_PER_DEGREE, 0, 0, 0);
//            activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1500, 1000);
        }

        @Override
        public void update() {
            super.update();


//            double profileTime = timer.seconds() - profileStart;
//
//            if (profileTime > activeProfile.duration()) {
//                MotionState start = new MotionState(opMode.motorUtility.getEncoderValue(Motors.TURRET), 0, 0, 0);
//                MotionState goal = new MotionState(opMode.motorUtility.getEncoderValue(Motors.TURRET) + calculateTurn() * Configuration.TURRET_TICKS_PER_DEGREE, 0, 0, 0);
//                activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1500, 1000);
//                profileStart = timer.seconds();
//            }
//
//            MotionState motionState = activeProfile.get(profileTime);
//
//            controller.setTargetPosition(motionState.getX());
//            controller.setTargetVelocity(motionState.getV());
//            controller.setTargetAcceleration(motionState.getA());

//            opMode.motorUtility.setPower(Motors.TURRET, controller.update(opMode.motorUtility.getEncoderValue(Motors.TURRET), opMode.motorUtility.getVelocity(Motors.TURRET)));

            opMode.motorUtility.setPower(Motors.TURRET, calculateTurn());

            if(CombinedTracker.trackType.equals(TrackType.POLE))
                lastTurretPos = opMode.motorUtility.getEncoderValue(Motors.TURRET);

            if(stateTimer.seconds() > 1.0)
                isDone = true;
        }

        private double calculateTurn() {
            if(opMode.visionDetection == null)
                return 0.0;

            Rect r = opMode.visionDetection.getRightPipeline().getBiggestCone();
            Rect l = opMode.visionDetection.getLeftPipeline().getBiggestCone();

            if(!(r.area() > 30))
                return 0.0;

            double coneCenterL = l.x + (l.width/2.0), coneCenterR = r.x + (r.width/2.0);
            double direction = coneCenterL < 320 - coneCenterR ? 1.0 : -1.0;
            double turn = Math.abs(coneCenterL - (320 - coneCenterR)) / 200.0;

            if(Math.abs(coneCenterL - (320 - coneCenterR)) < 3)
                return 0.0;

            return Range.clip(turn * 0.5, 0.0, 1.0) * direction;
        }

//        private double calculateTurn() {
//            Rect r = opMode.visionDetection.getLeftPipeline().getBiggestCone();
//
//            if(!(r.area() > 10))
//                return 0.0;
//
//            double coneCenter = r.x + (r.width/2.0);
//
//            if(coneCenter > (267.0 - 5) && coneCenter < (269.0 - 5)) {
//                isDone = true;
//                return 0.0;
//            }
//
//            double direction = coneCenter < (268.0 - 5) ? 1.0 : -1.0;
//            double turnDegrees = Math.abs(coneCenter - (268.0 - 5)) / 10.0;
//            return turnDegrees * direction;
//        }
    }

    class ArmPositionVisionDelayedIntake extends Executive.StateBase<AutoOpmode> {
        private final Configuration.ServoPosition servoPosition;
        private final double delay, intakePosition;
        private double prevArmPos;
        ArmPositionVisionDelayedIntake(Configuration.ServoPosition servoPosition, double delay, double intakePosition) {
            this.servoPosition = servoPosition;
            this.delay = delay;
            this.intakePosition = intakePosition;

            prevArmPos = servoPosition.getLeft();
        }

        @Override
        public void update() {
            super.update();

            if(timer.seconds() > delay)
                opMode.servoUtility.setAngle(Servos.CLAW, intakePosition);

            if(opMode.visionDetection == null)
                return;
            CombinedTracker l = opMode.visionDetection.getLeftPipeline(), r = opMode.visionDetection.getRightPipeline();
            if(l == null || r == null)
                return;
            // alpha
            double triangleRightAngle = 90 - r.getCameraAngle() + r.getObjectAngle();
            // beta
            double triangleLeftAngle = 90 + l.getCameraAngle() - l.getObjectAngle();

            double gamma = 180 - triangleLeftAngle - triangleRightAngle,
//                rightCameraDist = Math.sin(Math.toRadians(triangleLeftAngle)) * ((Configuration.CAMERA_DISTANCE_IN)/(Math.toRadians(gamma))),
                    leftCameraDist = Math.sin(Math.toRadians(triangleRightAngle)) *
                            ((Configuration.CAMERA_DISTANCE_IN)/Math.sin(Math.toRadians(gamma))),
                    frontDist = Math.abs(leftCameraDist * Math.sin(Math.toRadians(triangleLeftAngle)));
            frontDist = CombinedTracker.trackType.equals(TrackType.CONE) ? frontDist + Configuration.ARM_CONE_OFFSET_IN : frontDist;
            double pos = Double.parseDouble(RobotHardware.df.format((Math.toDegrees(Math.acos((frontDist - 3.0)/14.0)) + 90.0) / 180.0));

            try {
                opMode.servoUtility.setAngle(Servos.LEFT_ARM, Range.clip((pos + armOffset), 0.0, 1.0));
                opMode.servoUtility.setAngle(Servos.RIGHT_ARM, Range.clip(pos + armOffset, 0.0, 1.0));
                prevArmPos = Range.clip(pos + armOffset, 0.0, 1.0);
            } catch(IllegalArgumentException ignore) {
                opMode.servoUtility.setAngle(Servos.LEFT_ARM, prevArmPos);
                opMode.servoUtility.setAngle(Servos.RIGHT_ARM, prevArmPos);
            }
            lastArmPos = prevArmPos;
        }
    }

    static class ArmPositionDelayedIntake  extends Executive.StateBase<AutoOpmode> {
        private final double l,r;
        private final double delay, intakePosition;
        ArmPositionDelayedIntake(Configuration.ServoPosition servoPosition, double delay, double intakePosition) {
            l = servoPosition.getLeft();
            r = servoPosition.getRight();
            this.delay = delay;
            this.intakePosition = intakePosition;
        }

        ArmPositionDelayedIntake(double l, double r, double delay, double intakePosition) {
            this.l = l;
            this.r = r;
            this.delay = delay;
            this.intakePosition = intakePosition;
        }

        @Override
        public void update() {
            super.update();
            opMode.servoUtility.setAngle(Servos.LEFT_ARM, l);
            opMode.servoUtility.setAngle(Servos.RIGHT_ARM, r);
            if(timer.seconds() > delay)
                opMode.servoUtility.setAngle(Servos.CLAW, intakePosition);
        }
    }

    static class IntakeDelayedArmPosition  extends Executive.StateBase<AutoOpmode> {
        private final Configuration.ServoPosition servoPosition;
        private final double delay, intakePosition;
        IntakeDelayedArmPosition(Configuration.ServoPosition servoPosition, double delay, double intakePosition) {
            this.servoPosition = servoPosition;
            this.delay = delay;
            this.intakePosition = intakePosition;
        }

        @Override
        public void update() {
            super.update();
            opMode.servoUtility.setAngle(Servos.CLAW, intakePosition);
            if(timer.seconds() > delay) {
                opMode.servoUtility.setAngle(Servos.LEFT_ARM, servoPosition.getLeft());
                opMode.servoUtility.setAngle(Servos.RIGHT_ARM, servoPosition.getRight());
            }
        }
    }

    class TurretAngle extends Executive.StateBase<AutoOpmode> {
        private double angle;
        private double delay = 0;

        TurretAngle(double angle) {
            this.angle = angle;
        }

        TurretAngle(double angle, double delay) {
            this.angle = angle;
            this.delay = delay;
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            angle = angle * Configuration.TURRET_TICKS_PER_DEGREE;
        }

        @Override
        public void update() {
            super.update();
            isDone = opMode.motorUtility.goToPosition(Motors.TURRET, (int) angle, 1.0);
            if(isDone && delay > 0 && timer.seconds() > delay)
                nextState(TURRET, new TurretVisionTrack());
//            isDone = true;
        }
    }

    class SlidePosition extends Executive.StateBase<AutoOpmode> {
        private final double setpoint;
        private PIDFController controllerLeft, controllerRight;
        private MotionProfile activeProfile;
        private double profileStart;

        SlidePosition(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            controllerLeft = opMode.motorUtility.getController(Motors.SLIDE_LEFT);
            controllerRight = opMode.motorUtility.getController(Motors.SLIDE_RIGHT);
            profileStart = timer.seconds();
            MotionState start = new MotionState(opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT), 0, 0, 0);
            MotionState goal = new MotionState(setpoint, 0, 0, 0);
            activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1800, 1200);
        }

        @Override
        public void update() {
            super.update();

            double profileTime = timer.seconds() - profileStart;

            MotionState motionState = activeProfile.get(profileTime);

            controllerLeft.setTargetPosition(motionState.getX());
            controllerLeft.setTargetVelocity(motionState.getV());
            controllerLeft.setTargetAcceleration(motionState.getA());

            controllerRight.setTargetPosition(motionState.getX());
            controllerRight.setTargetVelocity(motionState.getV());
            controllerRight.setTargetAcceleration(motionState.getA());


            opMode.motorUtility.setPower(Motors.SLIDE_RIGHT, controllerRight.update(opMode.motorUtility.getEncoderValue(Motors.SLIDE_RIGHT), opMode.motorUtility.getVelocity(Motors.SLIDE_RIGHT)));
            opMode.motorUtility.setPower(Motors.SLIDE_LEFT, controllerLeft.update(opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT), opMode.motorUtility.getVelocity(Motors.SLIDE_LEFT)));

            isDone = (timer.seconds() - profileStart) > activeProfile.duration();
        }
    }

    static class Stop extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            for (Executive.StateMachine.StateType type : Executive.StateMachine.StateType.values())
                stateMachine.removeStateByType(type);
            opMode.stop();
        }
    }

    static class StopMotors extends  Executive.StateBase<AutoOpmode> {
        private final Motors[] motors;

        StopMotors(Motors... motors) {
            this.motors = motors;
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            for (Motors motor : motors)
                opMode.motorUtility.setPower(motor, 0);
        }
    }
}