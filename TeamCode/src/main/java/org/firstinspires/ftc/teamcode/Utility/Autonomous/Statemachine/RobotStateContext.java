package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
//import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Vision.DetectionAmount;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.SLIDE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.CAROUSEL;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_DRIVE_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_HIGH_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_LOW_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_MIDDLE_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_PICKUP_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.intakeExtend;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.intakeRetract;
import static org.firstinspires.ftc.teamcode.Utility.RobotHardware.df;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private ShippingHubLevel level = ShippingHubLevel.TOP;
//    private TrajectoryRR trajectoryRR;

    private static boolean parkOnly;
    private static boolean skipShippingHub;
    private static boolean skipCarousel;
    private static boolean skipSecondCargo = false;
    private static double delay = 0.0;

    enum ShippingHubLevel {
        TOP,
        MIDDLE,
        BOTTOM
    }

    public RobotStateContext(AutoOpmode opmode, AllianceColor allianceColor, StartPosition startPosition) {
        this.opmode = opmode;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(opmode);
        stateMachine.update();
    }

    public void init() {
//        trajectoryRR = new TrajectoryRR(opmode.mecanumDrive);
//        trajectoryRR.resetTrajectories(allianceColor);
        stateMachine.changeState(DRIVE, new Start());
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
        Pose2d poseEstimate = opmode.mecanumDrive.getPoseEstimate();
        opmode.telemetry.addData("X:        ", df.format(poseEstimate.getX()));
        opmode.telemetry.addData("Y:        ", df.format(poseEstimate.getY()));
        opmode.telemetry.addData("Heading:  ", df.format(Math.toDegrees(poseEstimate.getHeading())));
    }

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
//                trajectoryRR.resetTrajectories(AllianceColor.RED);
            switch (startPosition) {
                case CAROUSEL:
//                    setupInitialPosition(trajectoryRR.getStartCarousel());
                    nextState(DRIVE, new Initial());
                    break;
                case WAREHOUSE:
//                    setupInitialPosition(trajectoryRR.getStartWarehouse());
                    nextState(DRIVE, new Initial());
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
     * Initial State
     * State that is completely unnecessary.
     *
     * Trajectory: none
     * Next State: Scan
     */
    class Initial extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
//            nextState(SLIDE, new ArmDrive());
        }

        @Override
        public void update() {
            super.update();
            //Todo verify this is redundant
            if(!isDone) {
                isDone = true;
                stateTimer.reset();
            }
            if(stateTimer.seconds() > delay) {
                if (stateMachine.getStateReferenceByType(SLIDE).isDone)
                    nextState(DRIVE, new Scan());
            }
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
        @Override
        public void update() {
            super.update();
            // Nested switch statements, yay!
            switch (startPosition) {
                case CAROUSEL:
                    switch (opMode.initializationDetectionAmount) {
                        case LEFT:
                            level = ShippingHubLevel.BOTTOM;
                            break;
                        case RIGHT:
                            level = ShippingHubLevel.MIDDLE;
                            break;
                        case NONE:
                            // Redundant
                            level = ShippingHubLevel.TOP;
                    }
                    break;
                case WAREHOUSE:
                    switch (allianceColor) {
                        case BLUE:
                            switch (opMode.initializationDetectionAmount) {
                                case LEFT:
                                    level = ShippingHubLevel.MIDDLE;
                                    break;
                                case RIGHT:
                                    level = ShippingHubLevel.TOP;
                                    break;
                                case NONE:
                                    level = ShippingHubLevel.BOTTOM;
                            }
                            break;
                        case RED:
                            switch (opMode.initializationDetectionAmount) {
                                case LEFT:
                                    level = ShippingHubLevel.BOTTOM;
                                    break;
                                case RIGHT:
                                    level = ShippingHubLevel.MIDDLE;
                                    break;
                                case NONE:
                                    level = ShippingHubLevel.TOP;
                            }
                    }
            }

            switch (startPosition) {
                case CAROUSEL:
//                    nextState(DRIVE, new CarouselStartToHub());
                    break;
                case WAREHOUSE:
//                    nextState(DRIVE, new WarehouseStartToHub());
            }
        }
    }

//    class CarouselStartToHub extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            stateMachine.removeStateByType(SLIDE);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryCarouselStartToHub());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isBusy()) {
//                opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.CRADLE.getPos());
//                //Todo Make delay a variable
//                if (stateTimer.seconds() > 0.3) {
//                    opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
//                    //Todo Make this position a variable in Configuration.
//                    opMode.servoUtility.setAngle(Servos.CARGO_GATE, 0.0);
//                }
//                return;
//            }
//
//            if(!stateMachine.getCurrentStateByType(SLIDE).getSimpleName().equals(LiftHubLevel.class.getSimpleName()))
//                nextState(SLIDE, new LiftHubLevel(level));
//
//            if(stateMachine.getStateReferenceByType(SLIDE).isDone && opMode.mecanumDrive.isIdle()) {
//                opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.DROP.getPos());
//                opMode.servoUtility.setAngle(Servos.CARGO_GATE, 0.35);
//                if(!isDone) {
//                    stateTimer.reset();
//                    isDone = true;
//                }
//                //Todo Make delay a variable
//                if(stateTimer.seconds() > 1.5) {
//                    nextState(SLIDE, new ResetLiftToIntake());
//                    nextState(DRIVE, new HubToCarousel());
//                }
//
//            }
//        }
//    }
//
//    class WarehouseStartToHub extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            stateMachine.removeStateByType(SLIDE);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWarehouseStartToHub());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(!opMode.mecanumDrive.isIdle()) {
//                opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.CRADLE.getPos());
//                if (stateTimer.seconds() > 0.3) {
//                    opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
//                    //Todo Make this position a variable in Configuration.
//                    opMode.servoUtility.setAngle(Servos.CARGO_GATE, 0.0);
//                }
//                return;
//            }
//
//            if(!stateMachine.getCurrentStateByType(SLIDE).getSimpleName().equals(LiftHubLevel.class.getSimpleName()))
//                nextState(SLIDE, new LiftHubLevel(level));
//
//            if(stateMachine.getStateReferenceByType(SLIDE).isDone) {
//                opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.DROP.getPos());
//                opMode.servoUtility.setAngle(Servos.CARGO_GATE, 0.35);
//                if(!isDone) {
//                    stateTimer.reset();
//                    isDone = true;
//                }
//                if(stateTimer.seconds() > 0.75) {
//                    nextState(DRIVE, new HubToVerticalBarrier());
//                    nextState(SLIDE, new ResetLiftToIntake());
//                }
//
//            }
//        }
//    }
//
//    class HubToVerticalBarrier extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHubToVerticalBarrier());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isIdle()) {
//                nextState(DRIVE, new VerticalBarrierToWarehouse());
//            }
//        }
//    }
//
//    class VerticalBarrierToWarehouse extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryVerticalBarrierToWarehouse());
//            opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.INTAKE.getPos());
//            nextState(SLIDE, new IntakeCargo());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isBusy()) return;
//            if(!stateMachine.getCurrentStateByType(SLIDE).equals(PickupCargo.class)) {
//                nextState(SLIDE, new PickupCargo());
//            }
//            if(stateMachine.getStateReferenceByType(SLIDE).isDone)
//                if(skipSecondCargo)
//                    nextState(DRIVE, new WarehouseToShared());
//                else
//                    nextState(DRIVE, new WarehouseThroughVerticalBarrier());
//        }
//    }
//
//    class WarehouseToShared extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWarehouseParkToSharedAlign());
//            nextState(SLIDE, new ArmDrive());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isIdle())
//                nextState(DRIVE, new Stop());
//        }
//    }
//
//    class ArmDriveOpenBasket extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWarehouseParkToSharedAlign());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            isDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
//
//        }
//    }
//
//    class WarehouseThroughVerticalBarrier extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWarehouseThroughVerticalBarrier());
//            opMode.motorUtility.setPower(Motors.INTAKE, 0.0);
//            nextState(SLIDE, new ArmDrive());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isIdle())
//                nextState(DRIVE, new VerticalBarrierToHubAlign());
//        }
//    }
//
//    class VerticalBarrierToHubAlign extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryVerticalBarrierToHubAlign());
//            nextState(SLIDE, new LiftHubLevel(ShippingHubLevel.TOP));
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isIdle())
//                nextState(DRIVE, new SecondCargoHubAlignToDrop());
//        }
//    }
//
//    class SecondCargoHubAlignToDrop extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectorySecondCargoHubAlignToDrop());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isBusy()) return;
//            if(stateMachine.getStateReferenceByType(SLIDE).isDone) {
//                opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.DROP.getPos());
//                opMode.servoUtility.setAngle(Servos.CARGO_GATE, 0.35);
//                if(!isDone) {
//                    stateTimer.reset();
//                    isDone = true;
//                }
//                if(stateTimer.seconds() > 0.5) {
//                    nextState(DRIVE, new HubToWarehousePark());
//                }
//
//            }
//        }
//    }
//
//    class HubToWarehousePark extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHubToWarehousePark());
//            nextState(SLIDE, new ResetLiftToIntake());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isBusy()) return;
//            nextState(DRIVE, new Stop());
//        }
//    }
//
//    static class IntakeCargo extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void update() {
//            super.update();
//            opMode.motorUtility.setPower(Motors.INTAKE, 0.7);
//            opMode.servoUtility.setAngle(Servos.INTAKE, intakeExtend);
//            isDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_PICKUP_POS, 1.0);
//        }
//    }
//
//    static class PickupCargo extends Executive.StateBase<AutoOpmode> {
//        double intakeTime = 0.6;
//        @Override
//        public void update() {
//            super.update();
//            opMode.servoUtility.setAngle(Servos.INTAKE, intakeRetract + 0.1);
//            if(stateTimer.seconds() > intakeTime) {
//                isDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
//                opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.CRADLE.getPos());
//                opMode.servoUtility.setAngle(Servos.CARGO_GATE, 0.0);
//                opMode.motorUtility.setPower(Motors.INTAKE, -0.7);
//            }
//        }
//    }
//
//    class HubToCarousel extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHubToCarousel());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isBusy()) {
//                stateTimer.reset();
//                // Really janky way to prevent the robot from 'bouncing' off of the carousel and not spinning it.
//                //Todo make the X position be retrieved from the correct waypoint in TrajectoryRR.kt
//                if(allianceColor.equals(AllianceColor.BLUE) && opMode.mecanumDrive.getPoseEstimate().getX() < -56.0) {
//                    opMode.mecanumDrive.cancelFollowing();
//                    opMode.mecanumDrive.setDrivePower(new Pose2d());
//                }
//                return;
//            }
//            opMode.motorUtility.setPower(Motors.CAROUSEL, (allianceColor.equals(AllianceColor.BLUE) ? -1 : 1) * 0.6 * Range.clip(stateTimer.seconds(), 0.0, 1.0));
//
//            //Todo Make delay a variable
//            if(stateTimer.seconds() > 4.0) {
//                nextState(DRIVE, new CarouselToDepotPark());
//            }
//        }
//    }
//
//    class CarouselToDepotPark extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryCarouselToDepot());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isIdle()) {
//                nextState(DRIVE, new Stop());
//            }
//        }
//    }
//
//    static class ResetLiftToIntake extends Executive.StateBase<AutoOpmode> {
//        //Todo Decrease this time?
//        double craneTime = 1.6;
//        boolean aBoolean = false;
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.servoUtility.setAngle(Servos.BASKET, Configuration.ServoPosition.INTAKE.getPos());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            // Just let me die already, I wish to have no connections with this block of garbage.
//            //Todo what even is this
//            if(stateTimer.seconds() < craneTime)
//                opMode.servoUtility.setPower(ContinuousServo.CRANE, 1.0);
//            else {
//                opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
//                if(!isDone) {
//                    if(!aBoolean) {
//                        opMode.servoUtility.setPower(ContinuousServo.CRANE, 1.0);
//                        aBoolean = true;
//                    } else {
//                        opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
//                        isDone = true;
//                    }
//                }
//                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
//                //Todo Make this position a variable
//                opMode.servoUtility.setAngle(Servos.CARGO_GATE, 0.35);
//            }
//        }
//    }
//
//    static class LiftHubLevel extends Executive.StateBase<AutoOpmode> {
//        ShippingHubLevel hubLevel;
//        double craneTime = 1.65;
//        double gateTime = 0.5;
//        boolean armDone = false;
//
//        LiftHubLevel(ShippingHubLevel hubLevel) {
//            this.hubLevel = hubLevel;
//        }
//
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            if(hubLevel.equals(ShippingHubLevel.BOTTOM)) craneTime = 1.5;
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            switch (hubLevel) {
//                case TOP:
//                    armDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_HIGH_POS, 1.0);
//                    break;
//                case BOTTOM:
//                    armDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
//                    break;
//                case MIDDLE:
//                    armDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_MIDDLE_POS, 1.0);
//            }
//            if (stateTimer.seconds() < craneTime)
//                opMode.servoUtility.setPower(ContinuousServo.CRANE, -1.0);
//            else {
//                opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
//            }
//            isDone = stateTimer.seconds() > craneTime + gateTime && armDone;
//        }
//    }
//
//    /**
//     * State
//     *
//     *
//     * Trajectory: none
//     * Next State: Stop
//     */
//    class StartToCarousel extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartToCarousel());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(opMode.mecanumDrive.isIdle()){
//                if(!isDone) {
//                    stateTimer.reset();
//                    isDone = true;
//                    stateMachine.changeState(CAROUSEL, new CarouselRun(allianceColor.equals(AllianceColor.RED) ? 0.3:-0.3));
//                }
//                //Todo Make delay a variable
//                if(stateTimer.seconds()>4.0) {
//                    nextState(CAROUSEL, new Stop());
//                    stateMachine.changeState(DRIVE, new CarouselToHub());
//                }
//
//            }
//        }
//    }
//
//    /**
//     * State
//     *
//     *
//     * Trajectory: none
//     * Next State: Stop
//     */
//    class CarouselToHub extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryCarouselToHub());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//
//            if(opMode.mecanumDrive.isIdle())
//                nextState(DRIVE, new ResetForDriving());
//        }
//    }
//
//
//    /**
//     * Park State
//     *
//     *
//     * Trajectory: none
//     * Next State: Stop
//     */
//    class Park extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
////            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartToPark());
//        }
//
//        @Override
//        public void update() {
//            super.update();
//
//            if(opMode.mecanumDrive.isIdle()) {
//                nextState(DRIVE, new ResetForDriving());
//            }
//        }
//    }

//    /**
//     * Park State
//     *
//     *
//     * Trajectory: none
//     * Next State: Stop
//     */
//    class ResetForDriving extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            nextState(SLIDE, new ArmPickup());
//            stateMachine.removeStateByType(CAROUSEL);
//        }
//
//        @Override
//        public void update() {
//            super.update();
//
//            if(stateMachine.getStateReferenceByType(SLIDE).isDone)
//                nextState(DRIVE, new Stop());
//        }
//    }
//
//    static class Stop extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//            for (Executive.StateMachine.StateType type : Executive.StateMachine.StateType.values())
//                stateMachine.removeStateByType(type);
//            opMode.stop();
//        }
//    }

//    static class ArmPickup extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            isDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_PICKUP_POS, 1.0);
//        }
//    }
//
//    static class ArmDrive extends Executive.StateBase<AutoOpmode> {
//        @Override
//        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
//            super.init(stateMachine);
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            isDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
//        }
//    }
//
//    static class CarouselRun extends Executive.StateBase<AutoOpmode> {
//        double power;
//        CarouselRun(double power) {
//            this.power = power;
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            opMode.motorUtility.setPower(Motors.CAROUSEL, power);
//        }
//    }

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