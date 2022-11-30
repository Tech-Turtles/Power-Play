package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.INTAKE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive.StateMachine.StateType.TURRET;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.CLAW_OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.PIDController;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.opencv.core.Rect;


@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {
    
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double clawGrab = CLAW_OPEN;
    public static double linearSpeed = 0.75;
    public static double lateralSpeed = 0.75;
    public static double rotationSpeed = 1.0;
    public static double turretSpeed = 0.8;
    public static double slideMultiplier = 8.0;
    public static double turretMultiplier = 1.0;
    public static double reverseScale = 0.1;

    public static Configuration.ServoPosition armPosition = Configuration.ServoPosition.HOLD;

    private final PIDController pidController = new PIDController();
    public double setpoint = 0.0;

    private final Executive.StateMachine<Manual> stateMachine;
    private TrajectoryRR trajectoryRR;

    enum DriveMode {
        NORMAL_ROBOT_CENTRIC,
        NORMAL_FIELD_CENTRIC
    }

    private DriveMode currentDriveMode = DriveMode.NORMAL_FIELD_CENTRIC;

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
        trajectoryRR = new TrajectoryRR(this.mecanumDrive);

        motorUtility.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 12), Motors.SLIDE_LEFT, Motors.SLIDE_RIGHT);

        setpoint = 0.0;
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
    }

    @Override
    public void start() {
        super.start();
        mecanumDrive.setPoseEstimate(new Pose2d());
        pidController.init(0.005, 0.0, 0.0);
        stateMachine.changeState(DRIVE, new Drive_Manual());
        stateMachine.changeState(TURRET, new Turret_Manual());
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

            if(primary.BOnce())
                currentDriveMode = currentDriveMode == DriveMode.NORMAL_ROBOT_CENTRIC ? DriveMode.NORMAL_FIELD_CENTRIC : DriveMode.NORMAL_ROBOT_CENTRIC;

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
                            .rotated(-poseEstimate.getHeading());
                    driveDirection = new Pose2d(
                            robotFrameInput.getX(), robotFrameInput.getY(),
                            -primary.right_stick_x * rotationSpeed * precisionMode
                    );
                    break;
            }

            // Prevent slides from being powered if they are at a zero-like position.
            if(setpoint > 30.0) {
                motorUtility.setPower(Motors.SLIDE_LEFT, (motorUtility.getEncoderValue(Motors.SLIDE_LEFT) > setpoint + 15 ? reverseScale : 0.75) * pidController.update(setpoint, motorUtility.getEncoderValue(Motors.SLIDE_LEFT), statePeriod.seconds()));
                motorUtility.setPower(Motors.SLIDE_RIGHT, (motorUtility.getEncoderValue(Motors.SLIDE_RIGHT) > setpoint + 15 ? reverseScale : 0.75) * pidController.update(setpoint, motorUtility.getEncoderValue(Motors.SLIDE_RIGHT), statePeriod.seconds()));
            }

            mecanumDrive.setWeightedDrivePower(driveDirection);

            if(primary.X() && !stateMachine.getCurrentStateByType(TURRET).equals(Turret_Cone_Align.class)) {
                stateMachine.changeState(TURRET, new Turret_Cone_Align());
            } else if(!stateMachine.getCurrentStateByType(TURRET).equals(Turret_Manual.class) && !primary.X()) {
                stateMachine.changeState(TURRET, new Turret_Manual());
            }

            if(secondary.rightTriggerOnce())
                clawGrab = clawGrab == CLAW_CLOSED ? CLAW_OPEN : CLAW_CLOSED;

            servoUtility.setAngle(Servos.CLAW, clawGrab);

            setpoint += -secondary.right_stick_y * slideMultiplier;



            if(secondary.AOnce()) {
                setpoint = Configuration.INTAKE_POS;
            } else if(secondary.BOnce()) {
                setpoint = Configuration.MEDIUM_POS;
            } else if(secondary.XOnce()) {
                setpoint = Configuration.HIGH_POS;
            } else if(secondary.YOnce()) {
                setpoint = Configuration.LOW_POS;
            }

            if(secondary.rightBumperOnce() && !armPosition.equals(Configuration.ServoPosition.INTAKE)) {
                switch (armPosition) {
                    case START:
                        armPosition = Configuration.ServoPosition.HOLD;
                        break;
                    case HOLD:
                        armPosition = Configuration.ServoPosition.INTERMEDIARY;
                        break;
                    case INTERMEDIARY:
                        armPosition = Configuration.ServoPosition.INTAKE;
                }
                stateMachine.changeState(INTAKE, new Horizontal_Arm_Position(armPosition));
            } else if(secondary.leftBumperOnce() && !armPosition.equals(Configuration.ServoPosition.HOLD)) {
                switch (armPosition) {
                    case INTERMEDIARY:
                        armPosition = Configuration.ServoPosition.HOLD;
                        break;
                    case INTAKE:
                        armPosition = Configuration.ServoPosition.INTERMEDIARY;
                }
                stateMachine.changeState(INTAKE, new Horizontal_Arm_Position(armPosition));
            }
        }
    }

    class Horizontal_Arm_Position extends Executive.StateBase<Manual> {
        private final Configuration.ServoPosition servoPosition;
        Horizontal_Arm_Position(Configuration.ServoPosition servoPosition) {
            this.servoPosition = servoPosition;
        }

        @Override
        public void update() {
            super.update();
            servoUtility.setAngle(Servos.LEFT_ARM, servoPosition.getLeft());
            servoUtility.setAngle(Servos.RIGHT_ARM, servoPosition.getRight());
        }
    }

    //ToDo Implement distance estimation + driving & grabbing cone autonomously
    class Turret_Cone_Align extends Executive.StateBase<Manual> {

        @Override
        public void update() {
            super.update();
            motorUtility.setPower(Motors.TURRET, calculateTurn() * precisionMode);
        }

        private double calculateTurn() {
            if(visionDetection == null)
                return 0.0;

            Rect r = visionDetection.getPipeline().getBiggestCone();

            if(!(r.area() > 10))
                return 0.0;

            double coneCenter = r.x + (r.width/2.0);
            // Return if the cone is within tolerances (cone center is within 40 px of camera center)
            if(coneCenter > 318.0 && coneCenter < 322.0)
                return 0.0;

            double direction = coneCenter < 300.0 ? 1.0 : -1.0;
            double turn = Math.abs(coneCenter - 320) / 320.0;
            return Range.clip(turn * turretMultiplier, 0.0, 1.0) * direction;
        }
    }

    class Turret_Manual extends Executive.StateBase<Manual> {
        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            motorUtility.setPower(Motors.TURRET, -secondary.left_stick_x * turretSpeed);
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
