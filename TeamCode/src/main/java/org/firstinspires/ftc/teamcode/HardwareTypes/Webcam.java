package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum Webcam {

    VISION_RIGHT("vision right"),
    VISION_LEFT("vision left"),
    REALSENSE("realsense");

    final String name;

    Webcam(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}
