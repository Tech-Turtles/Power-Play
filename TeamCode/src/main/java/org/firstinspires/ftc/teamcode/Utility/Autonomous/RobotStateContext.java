package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.INTAKE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.SLIDE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.TURRET;
import static org.firstinspires.ftc.teamcode.Utility.RobotHardware.df;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.PIDController;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode autoOpmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private TrajectoryRR trajectoryRR;
    private Signal signal = Signal.NONE;
    private final PIDController pidController = new PIDController();
    private final double visionTimeout = 2.0;

    private final double slideRaiseMultiplier = 0.75;
    private final double slideLowerMultiplier = 0.1;

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

        autoOpmode.motorUtility.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 12), Motors.SLIDE_LEFT, Motors.SLIDE_RIGHT);
        pidController.init(0.005, 0.0, 0.0);
    }

    @Override
    public void update() {
        stateMachine.update();
        Pose2d poseEstimate = autoOpmode.mecanumDrive.getPoseEstimate();
        autoOpmode.telemetry.addData("Tag", autoOpmode.tagOfInterest.id);
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
            if(allianceColor.equals(AllianceColor.BLUE))
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

            signal = Signal.getSignalByOrdinal(autoOpmode.tagOfInterest == null ? 0 : autoOpmode.tagOfInterest.id);

            if(!signal.equals(Signal.NONE) || stateTimer.seconds() > visionTimeout) {
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
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartToHighPole());
            stateMachine.changeState(SLIDE, new SlidePosition(Configuration.HIGH_POS));
            stateMachine.changeState(INTAKE, new ArmPosition(Configuration.ServoPosition.HOLD));
            stateMachine.changeState(TURRET, new TurretAngle(45));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            if(!isDone && (stateMachine.getStateReferenceByType(SLIDE).isDone || timer.seconds() > 2.5) && (stateMachine.getStateReferenceByType(TURRET).isDone || timer.seconds() > 2.5)) {
                isDone = true;
                timer.reset();
                stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.INTERMEDIARY, 1.5, Configuration.CLAW_OPEN));
            }

            if(timer.seconds() > 2.0 && isDone) {
                stateMachine.changeState(DRIVE, new HighPoleToStack(1));
            }
        }
    }

    class HighPoleToStack extends Executive.StateBase<AutoOpmode> {
        private final int iteration;
        HighPoleToStack(int iteration) {
            this.iteration = iteration;
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHighPoleToStack());
            stateMachine.changeState(SLIDE, new SlidePosition(Configuration.MEDIUM_POS));
            stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.INTERMEDIARY, 0, Configuration.CLAW_OPEN));
            stateMachine.changeState(TURRET, new TurretAngle(180));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            if(!isDone) {
                isDone = true;
                stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.INTERMEDIARY, 0, Configuration.CLAW_CLOSED));
                timer.reset();
            }

            if(isDone && timer.seconds() > 0.25) {
                stateMachine.changeState(SLIDE, new SlidePosition(Configuration.MEDIUM_POS));
                if(timer.seconds() > 0.5)
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
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStackToHighPole());
            stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.HOLD, 0, Configuration.CLAW_CLOSED));
            stateMachine.changeState(SLIDE, new SlidePosition(Configuration.HIGH_POS));
            stateMachine.changeState(TURRET, new TurretAngle(45));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            if(!isDone && (stateMachine.getStateReferenceByType(SLIDE).isDone || timer.seconds() > 2.5) && (stateMachine.getStateReferenceByType(TURRET).isDone || timer.seconds() > 2.5)) {
                isDone = true;
                timer.reset();
                stateMachine.changeState(INTAKE, new ArmPositionDelayedIntake(Configuration.ServoPosition.INTERMEDIARY, 1.5, Configuration.CLAW_OPEN));
            }

            if(timer.seconds() > 2.0 && isDone) {
                if(iteration >= 6)
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
            stateMachine.changeState(TURRET, new TurretAngle(0));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

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

    static class ArmPositionDelayedIntake  extends Executive.StateBase<AutoOpmode> {
        private final Configuration.ServoPosition servoPosition;
        private final double delay, intakePosition;
        ArmPositionDelayedIntake(Configuration.ServoPosition servoPosition, double delay, double intakePosition) {
            this.servoPosition = servoPosition;
            this.delay = delay;
            this.intakePosition = intakePosition;
        }

        @Override
        public void update() {
            super.update();
            opMode.servoUtility.setAngle(Servos.LEFT_ARM, servoPosition.getLeft());
            opMode.servoUtility.setAngle(Servos.RIGHT_ARM, servoPosition.getRight());
            if(timer.seconds() > delay)
                opMode.servoUtility.setAngle(Servos.CLAW, intakePosition);
        }
    }

    static class TurretAngle extends Executive.StateBase<AutoOpmode> {
        private double angle;

        TurretAngle(double angle) {
            this.angle = angle;
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
//            isDone = true;
        }
    }

    class SlidePosition extends Executive.StateBase<AutoOpmode> {
        private final double setpoint;
        SlidePosition(double setpoint) {
            this.setpoint = setpoint;
        }
        @Override
        public void update() {
            super.update();
            opMode.motorUtility.setPower(Motors.SLIDE_LEFT,
                    (opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT) > setpoint + 15 ? slideLowerMultiplier : slideRaiseMultiplier)
                            * pidController.update(setpoint, opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT), timer.seconds()));
            opMode.motorUtility.setPower(Motors.SLIDE_RIGHT,
                    (opMode.motorUtility.getEncoderValue(Motors.SLIDE_RIGHT) > setpoint + 15 ? slideLowerMultiplier : slideRaiseMultiplier)
                            * pidController.update(setpoint, opMode.motorUtility.getEncoderValue(Motors.SLIDE_RIGHT), timer.seconds()));

            isDone = Math.abs(opMode.motorUtility.getEncoderValue(Motors.SLIDE_LEFT) - setpoint) < 20;
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