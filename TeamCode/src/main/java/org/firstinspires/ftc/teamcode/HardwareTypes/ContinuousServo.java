package org.firstinspires.ftc.teamcode.HardwareTypes;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public enum ContinuousServo {

    FRONT_LEFT("front left angle", DcMotorSimple.Direction.FORWARD, ServoTypes.INDEPENDENT),
    BACK_LEFT("back left angle", DcMotorSimple.Direction.FORWARD, ServoTypes.INDEPENDENT),
    BACK_RIGHT("back right angle", DcMotorSimple.Direction.REVERSE, ServoTypes.INDEPENDENT),
    FRONT_RIGHT("front right angle", DcMotorSimple.Direction.REVERSE, ServoTypes.INDEPENDENT);


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
