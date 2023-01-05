package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum ColorSensor {
    ;

    private final String name;

    ColorSensor(String name) {
        this.name = name;
    }

    public String getConfig() {
        return name;
    }
}
