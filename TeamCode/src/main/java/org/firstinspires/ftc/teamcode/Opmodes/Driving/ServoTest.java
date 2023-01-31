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

    public static double grab = 0.8, hold = 0.6, start = 0.3;

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
                servoUtility.setAngle(Servos.LEFT_ARM, grab);
                servoUtility.setAngle(Servos.RIGHT_ARM, grab);
                break;
            case HOLD:
                servoUtility.setAngle(Servos.LEFT_ARM, hold);
                servoUtility.setAngle(Servos.RIGHT_ARM, hold);
                break;
            case START:
                servoUtility.setAngle(Servos.LEFT_ARM, start);
                servoUtility.setAngle(Servos.RIGHT_ARM, start);
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
