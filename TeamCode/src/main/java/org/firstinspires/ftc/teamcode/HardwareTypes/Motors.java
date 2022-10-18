package org.firstinspires.ftc.teamcode.HardwareTypes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public enum Motors {

    FRONT_LEFT( "front left",   ExpansionHubs.HUB1, MotorTypes.DRIVE, DcMotorSimple.Direction.REVERSE, ZeroPowerBehavior.BRAKE, RunMode.RUN_USING_ENCODER),
    FRONT_RIGHT("front right",  ExpansionHubs.HUB1, MotorTypes.DRIVE, DcMotorSimple.Direction.FORWARD, ZeroPowerBehavior.FLOAT, RunMode.RUN_WITHOUT_ENCODER),
    BACK_LEFT(  "back left",    ExpansionHubs.HUB1, MotorTypes.DRIVE, DcMotorSimple.Direction.REVERSE, ZeroPowerBehavior.BRAKE, RunMode.RUN_USING_ENCODER),
    BACK_RIGHT( "back right",   ExpansionHubs.HUB1, MotorTypes.DRIVE, DcMotorSimple.Direction.FORWARD, ZeroPowerBehavior.BRAKE, RunMode.RUN_USING_ENCODER),
    TURRET("turret", ExpansionHubs.HUB2, MotorTypes.OTHER, DcMotorSimple.Direction.FORWARD, ZeroPowerBehavior.BRAKE, RunMode.RUN_USING_ENCODER);
    
    private final String configName;
    private final ExpansionHubs expansionHub;
    private final DcMotor.Direction direction;
    private final DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private final DcMotor.RunMode runMode;
    private final MotorTypes type;

    Motors(String configName, ExpansionHubs expansionHub, MotorTypes type,
           DcMotor.Direction direction, ZeroPowerBehavior zeroPowerBehavior, RunMode runMode) {
        this.configName = configName;
        this.expansionHub = expansionHub;
        this.direction = direction;
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.runMode = runMode;
        this.type = type;
    }

    public String getConfigName() {
        return configName;
    }

    public ExpansionHubs getExpansionHub() {
        return expansionHub;
    }

    public DcMotor.Direction getDirection() {
        return direction;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    public DcMotor.RunMode getRunMode() {
        return runMode;
    }

    public MotorTypes getType() {
        return type;
    }

    @Override
    public String toString() {
        return name();
    }
}
