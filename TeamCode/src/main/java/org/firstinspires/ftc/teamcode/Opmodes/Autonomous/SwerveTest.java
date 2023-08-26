package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.Utility.Math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Swerve.hardware.SwerveAbsoluteEncoder;

@Config
@Autonomous(name = "Swerve Test")
public class SwerveTest extends OpMode {
    PIDController pidController = new PIDController(0.02,0.0,0.0, 0.01);
    CRServoImplEx servo;
    SwerveAbsoluteEncoder encoder;
    public static double p = 0.02, d = 0, angle = 0;
    private double prevP = p, prevD = d;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    @Override
    public void init() {
        pidController.enableContinuousInput(0.0, 360.0);
        pidController.setTolerance(2.0);
        servo = hardwareMap.get(CRServoImplEx.class, ContinuousServo.FRONT_LEFT.getConfigName());
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPwmEnable();
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        encoder = new SwerveAbsoluteEncoder(hardwareMap.get(AnalogInput.class, "leftFrontEncoder"));
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        packet.put("Absolute Encoder", encoder.getCurrentPosition());
        double pid = pidController.calculate(encoder.getCurrentPosition(), angle);
//        if(!pidController.atSetpoint())
        servo.setPower(pid);
        if(p != prevP) {
            pidController.setP(p);
            prevP = p;
        }
        if(d != prevD) {
            pidController.setD(d);
            prevD = d;
        }
        dashboard.sendTelemetryPacket(packet);
        packet.clearLines();
    }
}
