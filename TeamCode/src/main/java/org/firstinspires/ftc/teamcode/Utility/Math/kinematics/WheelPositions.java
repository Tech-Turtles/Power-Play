package org.firstinspires.ftc.teamcode.Utility.Math.kinematics;

import org.firstinspires.ftc.teamcode.Utility.Math.interpolation.Interpolatable;

public interface WheelPositions<T extends WheelPositions<T>> extends Interpolatable<T> {
    /**
     * Returns a copy of this instance.
     *
     * @return A copy.
     */
    T copy();
}