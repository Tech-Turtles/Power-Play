package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum RevTouchSensors {
    ;

    private final String configName;

    RevTouchSensors(String configName) {
        this.configName = configName;
    }

    public String getConfigName() {
        return configName;
    }
}
