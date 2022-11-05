package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDController {

//    public static double Kp = 0.0;
//    public static double Ki = 0.0;
//    public static double Kd = 0.0;

    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    /* Derivative low-pass filter time constant */
    double tau;

    /* Output limits */
    double limMin = -1.0;
    double limMax = 1.0;

    /* Integrator limits */
    double limMinInt;
    double limMaxInt;

    /* Sample time (in seconds) */
    double T;

    /* Controller "memory" */
    double integrator;
    double prevError;			/* Required for integrator */
    double differentiator;
    double prevMeasurement;		/* Required for differentiator */

    public PIDController() {
        reset();
    }

    public PIDController(double Kp, double Ki, double Kd) {
        this();
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void init(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void reset() {
        integrator = 0.0;
        prevError = 0.0;
        differentiator = 0.0;
        prevMeasurement = 0.0;
        T = 0.0;
    }

    public double update(double setpoint, double measurement, double time) {
        double output;
        T = time;

        /*
         * Error signal
         */
        double error = setpoint - measurement;


        /*
         * Proportional
         */
        double proportional = Kp * error;

        /*
         * Integral
         */
        integrator = integrator + 0.5f * Ki * T * (error + prevError);

        /* Anti-wind-up via integrator clamping */
        if (integrator > limMaxInt) {

            integrator = limMaxInt;

        } else if (integrator < limMinInt) {

            integrator = limMinInt;

        }


        /*
         * Compute output and apply limits
         */
        output = proportional + integrator;

        if (output > limMax) {

            output = limMax;

        } else if (output < limMin) {

            output = limMin;

        }

        /* Store error and measurement for later use */
        prevError       = error;
        prevMeasurement = measurement;

        /* Return controller output */
        return output;
    }
}
