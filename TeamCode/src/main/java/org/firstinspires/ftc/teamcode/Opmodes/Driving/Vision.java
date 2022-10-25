package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        if(visionDetection != null) {
            if(visionDetection.getPipeline() != null)
                visionDetection.getPipeline().setTrackType(TrackType.SLEEVE);
        }
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

    }
}
