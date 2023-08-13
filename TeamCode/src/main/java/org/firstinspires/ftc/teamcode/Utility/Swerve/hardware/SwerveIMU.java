package org.firstinspires.ftc.teamcode.Utility.Swerve.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Translation3d;

import java.util.Optional;

/**
 * Swerve IMU abstraction to define a standard interface with a swerve drive.
 */
public abstract class SwerveIMU
{
    private boolean initialized = false;

    /**
     * Reset IMU to factory default.
     */
    public abstract void factoryDefault();

    /**
     * Clear sticky faults on IMU.
     */
    public abstract void clearStickyFaults();

    /**
     * Set the gyro offset.
     *
     * @param offset gyro offset as a {@link Rotation3d}.
     */
    public abstract void setOffset(Rotation3d offset);

    /**
     * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
     *
     * @return {@link Rotation3d} from the IMU.
     */
    public abstract Rotation3d getRawRotation3d();

    /**
     * Fetch the {@link Rotation3d} from the IMU. Robot relative.
     *
     * @return {@link Rotation3d} from the IMU.
     */
    public abstract Rotation3d getRotation3d();

    /**
     * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
     * empty.
     *
     * @return {@link Translation3d} of the acceleration as an {@link Optional}.
     */
    public abstract Optional<Translation3d> getAccel();

    public abstract void setIMU(BNO055IMUImpl imu);

    /**
     * Get the instantiated IMU object.
     *
     * @return IMU object.
     */
    public abstract Object getIMU();

    /**
     * Check if IMU has been initialized;
     * @return if IMU is initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Set the IMU to initialized
     */
    protected void initialize() {
        this.initialized = true;
    }
}
