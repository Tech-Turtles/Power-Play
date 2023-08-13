package org.firstinspires.ftc.teamcode.Utility.Math.kinematics;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.Utility.Math.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Math.interpolation.Interpolatable;

import java.util.Objects;

/** Represents the state of one swerve module. */
public class SwerveModulePosition
        implements Comparable<SwerveModulePosition>, Interpolatable<SwerveModulePosition> {
    /** Distance measured by the wheel of the module. */
    public double distanceMeters;

    /** Angle of the module. */
    public Rotation2d angle = Rotation2d.fromDegrees(0);

    /** Constructs a SwerveModulePosition with zeros for distance and angle. */
    public SwerveModulePosition() {}

    /**
     * Constructs a SwerveModulePosition.
     *
     * @param distanceMeters The distance measured by the wheel of the module.
     * @param angle The angle of the module.
     */
    public SwerveModulePosition(double distanceMeters, Rotation2d angle) {
        this.distanceMeters = distanceMeters;
        this.angle = angle;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof SwerveModulePosition) {
            SwerveModulePosition other = (SwerveModulePosition) obj;
            return Math.abs(other.distanceMeters - distanceMeters) < 1E-9 && angle.equals(other.angle);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(distanceMeters, angle);
    }

    /**
     * Compares two swerve module positions. One swerve module is "greater" than the other if its
     * distance is higher than the other.
     *
     * @param other The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModulePosition other) {
        return Double.compare(this.distanceMeters, other.distanceMeters);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format(
                "SwerveModulePosition(Distance: %.2f m, Angle: %s)", distanceMeters, angle);
    }

    /**
     * Returns a copy of this swerve module position.
     *
     * @return A copy.
     */
    public SwerveModulePosition copy() {
        return new SwerveModulePosition(distanceMeters, angle);
    }

    @Override
    public SwerveModulePosition interpolate(SwerveModulePosition endValue, double t) {
        return new SwerveModulePosition(
                MathUtil.interpolate(this.distanceMeters, endValue.distanceMeters, t),
                this.angle.interpolate(endValue.angle, t));
    }
}