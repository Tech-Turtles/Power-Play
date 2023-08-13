package org.firstinspires.ftc.teamcode.Utility.Swerve.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.Utility.Math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.PIDFConfig;

@Config
public class AngleServo extends SwerveServo {

    CRServoImplEx servo;
    SwerveAbsoluteEncoder encoder;
    PIDController pidController = new PIDController(0.006, 0.0, 0.0, 0.01);
    private double prevTime = 0, prevPos = 0;

    public AngleServo(ContinuousServo servo) {
        super(servo);
        pidController.setTolerance(1);
        pidController.enableContinuousInput(0.0, 360.0);
        prevTime = RobotHardware.totalTime;
    }

    @Override
    public void configurePIDF(PIDFConfig config) {

    }

    @Override
    public void configurePIDWrapping(double minInput, double maxInput) {

    }

    @Override
    public void setInverted(boolean inverted) {
        servo.setDirection(inverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void set(double percentOutput) {
        servo.setPower(percentOutput);
    }

    @Override
    public void setReference(double setpoint, double feedforward) {
        setReference(setpoint, feedforward, getPosition());
    }

    @Override
    public void setReference(double setpoint, double feedforward, double position) {
        pid = pidController.calculate(position, setpoint);
        if(pidController.atSetpoint())
            servo.setPower(0.0);
        else
            servo.setPower(pid);
    }

    @Override
    public double getVelocity() {
        if(prevTime == 0)
            return 0.0;
        double pos = getPosition();
        double time = RobotHardware.totalTime;
        double velo = (pos - prevPos) / (time - prevTime);
        prevPos = pos;
        prevTime = time;
        return velo;
    }

    @Override
    public double getPosition() {
        return encoder.getCurrentPosition();
    }

    @Override
    public void setPosition(double position) {

    }

    @Override
    public void setEncoder(SwerveAbsoluteEncoder encoder) {
        this.encoder = encoder;
        prevPos = getPosition();
    }

    @Override
    public void setServo(CRServoImplEx s) {
        this.servo = s;
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPwmEnable();
    }

    @Override
    public CRServoImplEx getServo() {
        return servo;
    }
}
