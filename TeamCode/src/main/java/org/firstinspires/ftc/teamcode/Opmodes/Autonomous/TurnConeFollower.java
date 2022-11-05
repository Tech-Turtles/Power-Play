package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.opencv.core.Rect;

@Autonomous(name = "cone turn test")
@Config
public class TurnConeFollower extends RobotHardware {

    public static double turn = 0.0;

    @Override
    public void init() {
        super.init();
        new Thread(this::loadVision).start();
        telemetry.addLine("Vision OpMode Initialized");
    }

    @Override
    public void init_loop() {
        super.init_loop();

        if (visionDetection == null)
            telemetry.addData("Vision:", "LOADING...");
        else
            telemetry.addData("Vision:", "INITIALIZED");
    }

    @Override
    public void loop() {
        super.loop();

        turn = calculateTurn();
        telemetry.addData("turn", turn);
        motorUtility.setPower(Motors.TURRET, turn);
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
