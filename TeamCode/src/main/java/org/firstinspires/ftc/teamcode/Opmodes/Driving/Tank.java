package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.MotorTypes;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@TeleOp(name="Tank Drive", group="E")
@Disabled
public class Tank extends RobotHardware {

    DcMotor fLeft;
    DcMotor bLeft;
    DcMotor fRight;
    DcMotor bRight;

    private final StringBuilder builder = new StringBuilder();

    @Override
    public void loop() {
        super.loop();
        builder.setLength(0);

        double leftPower  = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x, -1.0, 1.0) ;
        double rightPower = Range.clip(-gamepad1.right_stick_y + gamepad1.right_stick_x, -1.0, 1.0) ;

        fLeft = hardwareMap.get(DcMotor.class, "front left");
        bLeft = hardwareMap.get(DcMotor.class, "back left");
        fRight = hardwareMap.get(DcMotor.class, "front right");
        bRight = hardwareMap.get(DcMotor.class, "back right");

        fLeft.setPower(leftPower);
        fRight.setPower(rightPower);
        bLeft.setPower(leftPower);
        bRight.setPower(rightPower);

        if(primary.YOnce()) {
            fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (Motors motor : Motors.values()) {
            if(motor.getType() != MotorTypes.DRIVE) continue;
            builder.append(motor.name())
                    .append(": ")
                    .append(motorUtility.getEncoderValue(motor))
                    .append("\n");
        }

        telemetry.addLine(builder.toString());
    }
}