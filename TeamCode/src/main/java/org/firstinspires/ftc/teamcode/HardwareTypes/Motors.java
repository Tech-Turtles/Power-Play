package org.firstinspires.ftc.teamcode.HardwareTypes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public enum Motors {

    FRONT_LEFT( "front left",   ExpansionHubs.HUB2, MotorTypes.DRIVE, DcMotorSimple.Direction.REVERSE, ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER),
    FRONT_RIGHT("front right",  ExpansionHubs.HUB2, MotorTypes.DRIVE, DcMotorSimple.Direction.FORWARD, ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER),
    BACK_LEFT(  "back left",    ExpansionHubs.HUB2, MotorTypes.DRIVE, DcMotorSimple.Direction.REVERSE, ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER),
    BACK_RIGHT( "back right",   ExpansionHubs.HUB2, MotorTypes.DRIVE, DcMotorSimple.Direction.FORWARD, ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER),
    TURRET("turret", ExpansionHubs.HUB1, MotorTypes.OTHER, DcMotorSimple.Direction.FORWARD, ZeroPowerBehavior.BRAKE, RunMode.RUN_USING_ENCODER, new PIDCoefficients(0.0065, 0.0, 0.0001)),
    SLIDE_LEFT("slide left", ExpansionHubs.HUB1, MotorTypes.OTHER, DcMotorSimple.Direction.REVERSE, ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER, new PIDFController(new PIDCoefficients(0.0075, 0.0, 0.00005), 0.0008, 0.00005, 0.001)),
    SLIDE_RIGHT("slide right", ExpansionHubs.HUB1, MotorTypes.OTHER, DcMotorSimple.Direction.FORWARD, ZeroPowerBehavior.BRAKE, RunMode.RUN_WITHOUT_ENCODER, new PIDFController(new PIDCoefficients(0.0075, 0.0, 0.00005), 0.0008, 0.00005, 0.001));
    
    private final String configName;
    private final ExpansionHubs expansionHub;
    private final DcMotor.Direction direction;
    private final DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private final DcMotor.RunMode runMode;
    private final MotorTypes type;
    private final PIDCoefficients pidCoefficients;
    private PIDFController controller;

    Motors(String configName, ExpansionHubs expansionHub, MotorTypes type,
           DcMotor.Direction direction, ZeroPowerBehavior zeroPowerBehavior, RunMode runMode) {
        this.configName = configName;
        this.expansionHub = expansionHub;
        this.direction = direction;
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.runMode = runMode;
        this.type = type;
        this.pidCoefficients = null;
    }

    Motors(String configName, ExpansionHubs expansionHub, MotorTypes type,
           DcMotor.Direction direction, ZeroPowerBehavior zeroPowerBehavior, RunMode runMode, PIDCoefficients pidCoefficients) {
        this.configName = configName;
        this.expansionHub = expansionHub;
        this.direction = direction;
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.runMode = runMode;
        this.type = type;
        this.pidCoefficients = pidCoefficients;
    }

    Motors(String configName, ExpansionHubs expansionHub, MotorTypes type,
           DcMotor.Direction direction, ZeroPowerBehavior zeroPowerBehavior, RunMode runMode, PIDFController controller) {
        this.configName = configName;
        this.expansionHub = expansionHub;
        this.direction = direction;
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.runMode = runMode;
        this.type = type;
        this.pidCoefficients = null;
        this.controller = controller;
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

    public PIDCoefficients getPidCoefficients() {
        return pidCoefficients;
    }

    public void setController(PIDFController controller) {
        this.controller = controller;
    }

    public PIDFController getController() {
        return controller;
    }

    @Override
    public String toString() {
        return name();
    }
}
