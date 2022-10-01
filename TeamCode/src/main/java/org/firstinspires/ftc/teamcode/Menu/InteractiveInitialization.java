package org.firstinspires.ftc.teamcode.Menu;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;

public class InteractiveInitialization {

    private final Controller controller;
    private final Telemetry telemetry;
    private RobotHardware robot;
    private Gamepad gamepad;
    private ElapsedTimer timer = new ElapsedTimer();
    private final StringBuilder builder = new StringBuilder();

    private final double deadzone = 0.49f;

    private boolean locked = false;
    private int cursorIndex = 0;

    public InteractiveInitialization(RobotHardware robot) {
        this.robot = robot;
        this.gamepad = robot.gamepad1 == null ? robot.gamepad2 : robot.gamepad1;
        this.telemetry = robot.telemetry;
        this.controller = new Controller(gamepad);
    }

    public void loop() {
        builder.setLength(0);
    }

    private boolean updateInputs() {

        if(controller.leftStickButton()) {
           locked = !locked;
        }

        if(locked) return false;

        if(controller.left_stick_y > deadzone) {

        }

        return false;
    }
}
