package org.firstinspires.ftc.teamcode.Utility.Swerve.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Swerve.configuration.PIDFConfig;

public class DriveMotor extends SwerveMotor {

    private DcMotorEx motor;
    private double positionOffset = 0;

    public DriveMotor(Motors motor) {
        super(motor);
    }

    @Override
    public void configurePIDF(PIDFConfig config) {

    }

    @Override
    public void configurePIDWrapping(double minInput, double maxInput) {

    }

    @Override
    public void setMotorBrake(boolean isBrakeMode) {
        motor.setZeroPowerBehavior(isBrakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setDirection(inverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void set(double percentOutput) {
        motor.setPower(percentOutput);
    }

    @Override
    public void setReference(double setpoint, double feedforward) {

    }

    @Override
    public void setReference(double setpoint, double feedforward, double position) {

    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public double getPosition() {
        return motor.getCurrentPosition() - positionOffset;
    }

    @Override
    public void setPosition(double position) {
        positionOffset += position;
    }

    @Override
    public void setMotor(DcMotorEx m) {
        this.motor = m;
    }

    @Override
    public DcMotorEx getMotor() {
        return motor;
    }
}
