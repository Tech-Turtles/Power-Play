package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Webcam;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Vision.CombinedDetector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name = "April Tag Test")
public class AprilTag extends Manual {
    AprilTagDetector tagDetector;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void init() {
        super.init();

        new Thread(this::loadVisionPipeline).start();
    }

    @Override
    public void init_loop() {
        super.init_loop();

        if(tagDetector == null || tagDetector.getPipeline() == null)
            return;

        ArrayList<AprilTagDetection> currentDetections = tagDetector.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addData("Tag: ", tagOfInterest.id);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before");
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before");
            }

        }

        telemetry.update();
    }

    private void loadVisionPipeline() {
        tagDetector = new AprilTagDetector(hardwareMap, Webcam.VISION.getName());
        tagDetector.init();
    }
}
