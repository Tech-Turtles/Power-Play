package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Vision.TrackType;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.*;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_HIGH_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_MEDIUM_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_PICKUP_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.deadzone;
import static org.firstinspires.ftc.teamcode.Utility.RobotHardware.df;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode autoOpmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private TrajectoryRR trajectoryRR;
    private Signal signal = Signal.NONE;

    private static double visionTimeout = 2.0;

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
        stateMachine.init();
    }

    @Override
    public void update() {
        stateMachine.update();
        Pose2d poseEstimate = autoOpmode.mecanumDrive.getPoseEstimate();
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
    class Start extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            if(allianceColor.equals(AllianceColor.RED))
                trajectoryRR.resetTrajectories(AllianceColor.RED);

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
     *
     *
     * Trajectory: none
     * Next State:
     */
    class Scan extends Executive.StateBase<AutoOpmode> {
        boolean visionInitialized = false;

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            visionInitialized = false;
        }

        @Override
        public void update() {
            super.update();

            if(!visionInitialized)
                initializeVision();
            else
                signal = opMode.visionDetection.getPipeline().getSignalColor();

            if(!signal.equals(Signal.NONE) || stateTimer.seconds() > 20)
                nextState(DRIVE, new StartToSignalPark());
        }

        private void initializeVision() {
            if(opMode.visionDetection != null && opMode.visionDetection.getPipeline() != null) {
                opMode.visionDetection.getPipeline().setTrackType(TrackType.SLEEVE);
                visionInitialized = true;
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

    class SlidePosition extends Executive.StateBase<AutoOpmode> {
        private final int position;
        private final double delay;
        boolean reset;

        SlidePosition(int position, double delay) {
            this.position = position;
            this.delay = delay;
            reset = delay == 0.0;
        }

        @Override
        public void update() {
            super.update();
            if(!reset) {
                reset = true;
                stateTimer.reset();
            }
            if(stateTimer.seconds() > delay)
                isDone = opMode.motorUtility.goToPosition(Motors.SLIDE, position, 1.0);
        }
    }

    class HoldCone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.servoUtility.setAngle(Servos.GRAB_LEFT, 0.0);
            opMode.servoUtility.setAngle(Servos.GRAB_RIGHT, 0.0);
        }
    }

    class DropCone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.servoUtility.setAngle(Servos.GRAB_LEFT, 0.9);
            opMode.servoUtility.setAngle(Servos.GRAB_RIGHT, 0.9);
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