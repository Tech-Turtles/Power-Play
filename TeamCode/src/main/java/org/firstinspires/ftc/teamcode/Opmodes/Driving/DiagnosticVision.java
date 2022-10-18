package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Diagnostic Vision", group="C")
public class DiagnosticVision extends Diagnostic {

    @Override
    public void init() {
        super.init();
        new Thread(this::loadVision).start();
        telemetry.addLine("Diagnostic Vision OpMode Initialized");
    }

    @Override
    public void init_loop(){
        super.init_loop();
        if (visionDetection == null)
            telemetry.addData("Vision: ", "LOADING...");
        else
            telemetry.addData("Vision: ", "INITIALIZED");

    }

    @Override
    public void loop() {
        super.loop();

    }
}
