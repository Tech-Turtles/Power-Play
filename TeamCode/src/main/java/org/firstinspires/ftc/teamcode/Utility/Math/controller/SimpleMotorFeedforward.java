package org.firstinspires.ftc.teamcode.Utility.Math.controller;

import org.firstinspires.ftc.teamcode.Utility.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utility.Math.Nat;
import org.firstinspires.ftc.teamcode.Utility.Math.numbers.N1;
import org.firstinspires.ftc.teamcode.Utility.Math.system.LinearSystem;
import org.firstinspires.ftc.teamcode.Utility.Math.system.plant.LinearSystemId;

/** A helper class that computes feedforward outputs for a simple permanent-magnet DC motor. */
public class SimpleMotorFeedforward {
    public final double ks;
    public final double kv;
    public final double ka;

    /**
     * Creates a new SimpleMotorFeedforward with the specified gains. Units of the gain values will
     * dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     */
    public SimpleMotorFeedforward(double ks, double kv, double ka) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }

    /**
     * Creates a new SimpleMotorFeedforward with the specified gains. Acceleration gain is defaulted
     * to zero. Units of the gain values will dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kv The velocity gain.
     */
    public SimpleMotorFeedforward(double ks, double kv) {
        this(ks, kv, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param velocity The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param currentVelocity The current velocity setpoint.
     * @param nextVelocity The next velocity setpoint.
     * @param dtSeconds Time between velocity setpoints in seconds.
     * @return The computed feedforward.
     */
    public double calculate(double currentVelocity, double nextVelocity, double dtSeconds) {
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(this.kv, this.ka);
        LinearPlantInversionFeedforward<N1, N1, N1> feedforward = new LinearPlantInversionFeedforward<>(plant, dtSeconds);

        Matrix<N1, N1> r = Matrix.mat(Nat.N1(), Nat.N1()).fill(currentVelocity);
        Matrix<N1, N1> nextR = Matrix.mat(Nat.N1(), Nat.N1()).fill(nextVelocity);

        return ks * Math.signum(currentVelocity) + feedforward.calculate(r, nextR).get(0, 0);
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
     * zero).
     *
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
     * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
     * simultaneously achievable - enter the acceleration constraint, and this will give you a
     * simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the motor.
     * @param acceleration The acceleration of the motor.
     * @return The maximum possible velocity at the given acceleration.
     */
    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - ks - acceleration * ka) / kv;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
     * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
     * simultaneously achievable - enter the acceleration constraint, and this will give you a
     * simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the motor.
     * @param acceleration The acceleration of the motor.
     * @return The minimum possible velocity at the given acceleration.
     */
    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + ks - acceleration * ka) / kv;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
     * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
     * simultaneously achievable - enter the velocity constraint, and this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the motor.
     * @param velocity The velocity of the motor.
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - ks * Math.signum(velocity) - velocity * kv) / ka;
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage supply and a velocity.
     * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
     * simultaneously achievable - enter the velocity constraint, and this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the motor.
     * @param velocity The velocity of the motor.
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, velocity);
    }
}
