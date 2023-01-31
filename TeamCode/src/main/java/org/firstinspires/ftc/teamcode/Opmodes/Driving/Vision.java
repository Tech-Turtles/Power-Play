package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.CombinedDetector;
import org.firstinspires.ftc.teamcode.Vision.CombinedTracker;
import org.firstinspires.ftc.teamcode.Vision.TrackType;

@TeleOp(name="Vision", group="B")
@Config
public class Vision extends Manual {

    public static double armOffset = -0.1;
    private double prevArmPosition = Configuration.ServoPosition.HOLD.getLeft();

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
        double triangleRightAngle = 90 - r.getCameraAngle() + r.getObjectAngle();
        // beta
        double triangleLeftAngle = 90 + l.getCameraAngle() - l.getObjectAngle();

        double gamma = 180 - triangleLeftAngle - triangleRightAngle,
//                rightCameraDist = Math.sin(Math.toRadians(triangleLeftAngle)) * ((Configuration.CAMERA_DISTANCE_IN)/(Math.toRadians(gamma))),
                leftCameraDist = Math.sin(Math.toRadians(triangleRightAngle)) *
                        ((Configuration.CAMERA_DISTANCE_IN)/Math.sin(Math.toRadians(gamma))),
                frontDist = Math.abs(leftCameraDist * Math.sin(Math.toRadians(triangleLeftAngle)));

        telemetry.addData("Distance:", frontDist);
        telemetry.addData("triangleRightAngle:", triangleRightAngle);
        telemetry.addData("triangleLeftAngle:", triangleLeftAngle);
//        telemetry.addData("gamma:", gamma);
//        telemetry.addData("leftCameraDist", leftCameraDist);
//        telemetry.addData("right Pole Angle:", r.getPoleAngle());
//        telemetry.addData("left Pole Angle", l.getPoleAngle());
//        telemetry.addData("left pole pixels", ((l.getBiggestCone().x + (l.getBiggestCone().width/2.0) - 160.0) / 6.23333));
//        telemetry.addData("right  pole pixels", ((r.getBiggestCone().x + (r.getBiggestCone().width/2.0) - 160.0) / 6.23333));
//        packet.put("Distance", frontDist);

//        double armPosition = Double.parseDouble(RobotHardware.df.format((Math.toDegrees(Math.acos((frontDist - 3.0)/14.0)) + 90.0) / 180.0));
//        telemetry.addData("Arm pos:",armPosition);
//        if(secondary.left_trigger > 0.49) {
//            try {
//                servoUtility.setAngle(Servos.LEFT_ARM, Range.clip((armPosition + armOffset), 0.0, 1.0));
//                servoUtility.setAngle(Servos.RIGHT_ARM, Range.clip(armPosition + armOffset, 0.0, 1.0));
//                prevArmPosition = Range.clip(armPosition + armOffset, 0.0, 1.0);
//                telemetry.addData("Vision Error:", "none");
//            } catch(IllegalArgumentException ignore) {
//                servoUtility.setAngle(Servos.LEFT_ARM, prevArmPosition);
//                servoUtility.setAngle(Servos.RIGHT_ARM, prevArmPosition);
//                telemetry.addData("Vision Error:", "invalid servo position");
//            }
//        }
    }
}
