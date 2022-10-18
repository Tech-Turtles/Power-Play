package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Webcam;
import org.firstinspires.ftc.teamcode.Vision.SignalSleeveDetector;

@TeleOp(name="Signal Sleeve", group="B")
@Config
public class SignalSleeve extends Manual {
    SignalSleeveDetector sleeveTracker;

    @Override
    public void init() {
        super.init();
        new Thread(() -> {
            sleeveTracker = new SignalSleeveDetector(hardwareMap, Webcam.VISION.getName());
            sleeveTracker.init();
        }).start();

        telemetry.addLine("Signal Sleeve OpMode Initialized");
    }

    @Override
    public void init_loop(){
        super.init_loop();
        if (sleeveTracker == null)
            telemetry.addData("Vision:", "LOADING...");
        else
            telemetry.addData("Vision:", "INITIALIZED");
    }

    @Override
    public void loop() {
        super.loop();
        if(sleeveTracker != null) {
            telemetry.addData("Signal Color: ", sleeveTracker.getColor());
            telemetry.addData("Signal Ordinal: ", sleeveTracker.getOrdinal());
        }
    }
}
