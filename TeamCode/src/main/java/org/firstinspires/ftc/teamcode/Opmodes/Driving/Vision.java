package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        if (coneDetector == null)
            telemetry.addData("Vision:", "LOADING...");
        else
            telemetry.addData("Vision:", "INITIALIZED");
    }

    @Override
    public void loop() {
        super.loop();

    }
}
