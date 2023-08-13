package org.firstinspires.ftc.teamcode.Utility.Swerve.hardware;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Quaternion;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Translation3d;

import java.util.Optional;

public class BNO055 extends SwerveIMU {

    BNO055IMUImpl gyro;

    private Rotation3d offset = new Rotation3d();

    @Override
    public void factoryDefault() {

    }

    @Override
    public void clearStickyFaults() {

    }

    @Override
    public void setOffset(Rotation3d offset) {
        this.offset = offset;
    }

    @Override
    public Rotation3d getRawRotation3d() {
        org.firstinspires.ftc.robotcore.external.navigation.Quaternion q = gyro.getQuaternionOrientation();
        return new Rotation3d(new Quaternion(q.w, q.x, q.y, q.z));
    }

    @Override
    public Rotation3d getRotation3d() {
        return getRawRotation3d().minus(offset);
    }

    @Override
    public Optional<Translation3d> getAccel() {
        Acceleration a = gyro.getAcceleration();
        return Optional.of(
                new Translation3d(a.xAccel, a.yAccel, a.zAccel).times(9.81));
    }

    @Override
    public void setIMU(BNO055IMUImpl imu) {
        this.gyro = imu;
        initialize();
    }

    @Override
    public Object getIMU() {
        return gyro;
    }
}
