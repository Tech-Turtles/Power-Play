package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Vision.CombinedDetector;
import org.firstinspires.ftc.teamcode.Vision.CombinedTracker;
import org.firstinspires.ftc.teamcode.Vision.TrackType;

@TeleOp(name="Vision", group="B")
@Config
public class Vision extends Manual {

    @Override
    public void init() {
        super.init();
        new Thread(this::loadVision).start();
        telemetry.addLine("Vision OpMode Initialized");
    }

    @Override
    public void init_loop(){
        super.init_loop();
        if (visionDetection == null)
            telemetry.addData("Vision:", "LOADING...");
        else
            telemetry.addData("Vision:", "INITIALIZED");

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        if(visionDetection == null)
            return;

        CombinedTracker l = visionDetection.getLeftPipeline(), r = visionDetection.getRightPipeline();
        if(l == null || r == null)
            return;
        // alpha
        double triangleRightAngle = 90 + r.getCameraAngle() + r.getPoleAngle();
        // beta
        double triangleLeftAngle = 90 - l.getCameraAngle() - l.getPoleAngle();

        double gamma = 180 - triangleLeftAngle - triangleRightAngle,
//                rightCameraDist = Math.sin(leftAngle) * ((Configuration.CAMERA_DISTANCE_IN)/(objectAngle)),
                leftCameraDist = Math.sin(Math.toRadians(triangleRightAngle)) *
                        ((Configuration.CAMERA_DISTANCE_IN)/Math.sin(Math.toRadians(gamma))),
                frontDist = leftCameraDist * Math.sin(Math.toRadians(triangleLeftAngle));
        telemetry.addData("Distance:", frontDist);
        telemetry.addData("triangleRightAngle:", triangleRightAngle);
        telemetry.addData("triangleLeftAngle:", triangleLeftAngle);
        telemetry.addData("gamma:", gamma);
        telemetry.addData("leftCameraDist", leftCameraDist);
        telemetry.addData("right Pole Angle:", r.getPoleAngle());
        telemetry.addData("left Pole Angle", l.getPoleAngle());
        packet.put("Distance", frontDist);

    }
}
