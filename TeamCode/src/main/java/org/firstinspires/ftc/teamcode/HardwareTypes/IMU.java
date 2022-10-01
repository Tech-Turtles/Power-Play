package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum IMU {

    IMU1("IMU_1"),
    IMU2("IMU_2");

    private final String name;

    IMU(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}
