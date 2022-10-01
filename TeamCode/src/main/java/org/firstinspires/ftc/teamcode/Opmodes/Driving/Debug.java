package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.HardwareTypes.IMU;
import org.firstinspires.ftc.teamcode.HardwareTypes.MotorTypes;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.BNO055IMUUtil;

/**
 * @author Christian
 * An opmode that shows data about commonly used things, including motors, servos, IMU.
 */
@TeleOp(name="Debug", group="B")
public class Debug extends Manual {

    @Override
    public void init() {
        super.init();
        imuUtil = new IMUUtilities(this, IMU.IMU1.getName(), IMUUtilities.ImuMode.FAST_HEADING_ONLY);
        telemetry.addLine("----Debug Initialized----");
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        for (MotorTypes type : MotorTypes.values()) {
            telemetry.addLine(type.name());
            for (Motors motor : Motors.values()) {
                if(type != motor.getType()) continue;
                telemetry.addData(motor.name() + ": ", motorUtility.getEncoderValue(motor));
            }
        }

        for (Servos servo : Servos.values()) {
            telemetry.addData(servo.name() + ": ", servoUtility.getAngle(servo));
        }

        telemetry.addData("Period Average: ", df_precise.format(period.getAveragePeriodSec()) + "s");
        telemetry.addData("Period Max:     ", df_precise.format(period.getMaxPeriodSec()) + "s");

        if(imuUtil.imu != null) {
            imuUtil.updateNow();
            imuUtil.displayTelemetry();
        }
    }
}