package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum RevDistanceSensor {
    CLAW_DISTANCE("claw distance");

    private final String configName;

    RevDistanceSensor(String configName) {
        this.configName = configName;
    }

    public String getConfigName() {
        return configName;
    }
}
