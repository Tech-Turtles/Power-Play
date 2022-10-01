package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.opencv.core.Rect;

@Autonomous(name = "cone turn test")
public class TurnConeFollower extends RobotHardware {

    ElapsedTimer timer;

    @Override
    public void init() {
        super.init();
        timer = new ElapsedTimer();
        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        new Thread(this::loadVision).start();
        telemetry.addLine("Vision OpMode Initialized");
    }

    @Override
    public void init_loop() {
        super.init_loop();

        if (coneDetector == null)
            telemetry.addData("Vision:", "LOADING...");
        else
            telemetry.addData("Vision:", "INITIALIZED");
    }

    @Override
    public void start() {
        super.start();
        timer.reset();
        turn = calculateTurn();
    }

    double turn;

    @Override
    public void loop() {
        super.loop();
        mecanumDrive.update(packet);

        if(coneDetector != null) {
            telemetry.addData("turn", calculateTurn());
            mecanumDrive.setWeightedDrivePower(new Pose2d(0,0,calculateTurn()));
        }
    }

    private double calculateTurn() {
        if(coneDetector == null)
            return 0;

        Rect r = coneDetector.getBiggestCone();
        double coneCenter = r.x + (r.width/2.0);
        // Return if the cone is within tolerances (cone center is within 40 px of camera center)
        if(coneCenter > 280 && coneCenter < 320)
            return 0;

        double direction = coneCenter < 280 ? 1.0 : -1.0;
        double turn = Math.abs(coneCenter - 320);
        switch ((int) (turn / 100)) {
            case 0:
                return 0.1 * direction;
            case 1:
                return 0.3 * direction;
            case 2:
                return 0.6 * direction;
            default:
                return 1.0 * direction;
        }
    }
}
