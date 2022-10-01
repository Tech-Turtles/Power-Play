package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum Webcam {

    VISION("vision"),
    REALSENSE("realsense");

    final String name;

    Webcam(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}
