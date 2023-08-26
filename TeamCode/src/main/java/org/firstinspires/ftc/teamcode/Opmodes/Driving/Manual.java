package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import static org.firstinspires.ftc.teamcode.Utility.Configuration.precisionPercentage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;


@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double linearSpeed = 1.0, lateralSpeed = 1.0, rotationSpeed = 1.0;
    private double precisionMode = 1.0;

    double theta = Math.toRadians(0.0);

    public static Pose2d startingPosition = new Pose2d();

    private final Executive.StateMachine<Manual> stateMachine;


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
        stateMachine.init();

//        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
    }

    @Override
    public void start() {
        super.start();
//        mecanumDrive.setPoseEstimate(startingPosition);
//        stateMachine.changeState(DRIVE, new Drive_Manual());
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
//        mecanumDrive.update(packet);
//        displayTelemetry();
    }
    
    class Drive_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            if (primary.AOnce())
                precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;

            if (primary.YOnce())
                mecanumDrive.setPoseEstimate(new Pose2d(mecanumDrive.getPoseEstimate().vec(), 0.0));

            if (primary.BOnce())
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
                            .rotated(Math.toRadians(AutoOpmode.robotStartPos.equals(StartPosition.FAR) ? (AutoOpmode.robotColor.equals(AllianceColor.RED) ? (270.0) : (90.0)) : (AutoOpmode.robotColor.equals(AllianceColor.RED) ? (90.0) : (270.0))));
                    driveDirection = new Pose2d(
                            robotFrameInput.getX(), robotFrameInput.getY(),
                            -primary.right_stick_x * rotationSpeed * precisionMode
                    );
                    theta = mecanumDrive.getPoseEstimate().getHeading();
                    break;
            }

            mecanumDrive.setWeightedDrivePower(driveDirection);
        }
    }

    void displayTelemetry() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

        telemetry.addData("Precision mode:      ", df.format(precisionMode));
        telemetry.addData("X:                   ", df.format(poseEstimate.getX()));
        telemetry.addData("Y:                   ", df.format(poseEstimate.getY()));
        telemetry.addData("Heading:             ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        if (packet != null) {
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
