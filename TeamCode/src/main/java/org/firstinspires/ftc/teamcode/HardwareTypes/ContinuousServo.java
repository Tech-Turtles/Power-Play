package org.firstinspires.ftc.teamcode.HardwareTypes;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public enum ContinuousServo {

    CLOSE_INTAKE("intake close", DcMotorSimple.Direction.FORWARD, ServoTypes.INDEPENDENT),
    FAR_INTAKE("intake far", DcMotorSimple.Direction.FORWARD, ServoTypes.INDEPENDENT);

    private final String configName;
    private final DcMotorSimple.Direction direction;
    private final ServoTypes type;

    ContinuousServo(String configName, DcMotorSimple.Direction direction, ServoTypes type) {
        this.configName = configName;
        this.direction = direction;
        this.type = type;
    }

    public String getConfigName() {
        return configName;
    }

    public DcMotorSimple.Direction getDirection() {
        return direction;
    }

    public ServoTypes getType() {
        return type;
    }
}
