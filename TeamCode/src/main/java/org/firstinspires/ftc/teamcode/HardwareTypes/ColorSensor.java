package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum ColorSensor {
    ;

    String name;

    ColorSensor(String name) {
        this.name = name;
    }

    public String getConfig() {
        return name;
    }
}
