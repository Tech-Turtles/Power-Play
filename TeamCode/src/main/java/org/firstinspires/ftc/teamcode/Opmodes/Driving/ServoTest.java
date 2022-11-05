package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@Config
@TeleOp(name = "ServoTest", group = "E")
public class ServoTest extends RobotHardware {
    public static double left = 0.0;
    public static double right = 0.0;

    public static double grabL = 0.8;
    public static double grabR = 0.8;

    public static double holdL = 0.6;
    public static double holdR = 0.6;

    public static double startL = 0.34;
    public static double startR = 0.35;

    public static ServoState servoState = ServoState.START;

    enum ServoState {
        POSITION,
        GRAB,
        START,
        HOLD
    }

    @Override
    public void init() {
        super.init();
        servoUtility.setAngle(Servos.LEFT_ARM, servoUtility.getAngle(Servos.LEFT_ARM));
        servoUtility.setAngle(Servos.RIGHT_ARM, servoUtility.getAngle(Servos.RIGHT_ARM));
    }

    @Override
    public void loop() {
        super.loop();

        switch (servoState) {
            case POSITION:
                servoUtility.setAngle(Servos.LEFT_ARM, left);
                servoUtility.setAngle(Servos.RIGHT_ARM, right);
                break;
            case GRAB:
                servoUtility.setAngle(Servos.LEFT_ARM, grabL);
                servoUtility.setAngle(Servos.RIGHT_ARM, grabR);
                break;
            case HOLD:
                servoUtility.setAngle(Servos.LEFT_ARM, holdL);
                servoUtility.setAngle(Servos.RIGHT_ARM, holdR);
                break;
            case START:
                servoUtility.setAngle(Servos.LEFT_ARM, startL);
                servoUtility.setAngle(Servos.RIGHT_ARM, startR);
                break;
        }

        if(primary.AOnce())
            servoState = ServoState.POSITION;
        else if(primary.BOnce())
            servoState = ServoState.GRAB;
        else if(primary.XOnce())
            servoState = ServoState.HOLD;
        else if(primary.YOnce())
            servoState = ServoState.START;
    }
}
