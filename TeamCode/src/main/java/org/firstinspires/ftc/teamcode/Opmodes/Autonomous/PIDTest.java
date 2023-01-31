package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@TeleOp
@Config
public class PIDTest extends RobotHardware {

    PIDCoefficients coeffs;
    PIDFController controller;
    NanoClock clock;
    MotionProfile activeProfile;
    boolean movingForwards;
    double profileStart;

    public static double kP = 0.003, kI = 0.00025, kD = 0.0003, kV = 0.001, kA = 0.0, kStatic = 0.001, setpoint = 600.0, targetVelo = 1200, targetAccel = 800;

    private double prevKP = 0, prevKI = 0, prevKD = 0, prevKV = 0.0, prevKA = 0.0, prevKStatic = 0.0;

    @Override
    public void init() {
        super.init();
        coeffs = new PIDCoefficients(kP, kI, kD);
        controller = new PIDFController(coeffs, kV, kA, kStatic);
//        controller = new PIDFController(coeffs);
        controller.setTargetVelocity(targetVelo);
        controller.setTargetAcceleration(targetAccel);
    }

    @Override
    public void start() {
        super.start();
        clock = NanoClock.system();
        movingForwards = true;
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(setpoint, 0, 0, 0);
        activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, targetVelo, targetAccel);
        profileStart = clock.seconds();
    }

    @Override
    public void loop() {
        super.loop();


        // calculate and set the motor power
        double profileTime = clock.seconds() - profileStart;

        if (profileTime > activeProfile.duration()) {
            // generate a new profile
            movingForwards = !movingForwards;
            activeProfile = generateProfile(movingForwards);
            profileStart = clock.seconds();
        }

        MotionState motionState = activeProfile.get(profileTime);

        controller.setTargetPosition(motionState.getX());
        controller.setTargetVelocity(motionState.getV());
        controller.setTargetAcceleration(motionState.getA());


        motorUtility.setPower(Motors.SLIDE_RIGHT, controller.update(motorUtility.getEncoderValue(Motors.SLIDE_RIGHT), motorUtility.getVelocity(Motors.SLIDE_RIGHT)));
        motorUtility.setPower(Motors.SLIDE_LEFT, controller.update(motorUtility.getEncoderValue(Motors.SLIDE_LEFT), motorUtility.getVelocity(Motors.SLIDE_LEFT)));

        packet.put("Motor Velo R", motorUtility.getVelocity(Motors.SLIDE_RIGHT));
        packet.put("Motor Velo L", motorUtility.getVelocity(Motors.SLIDE_LEFT));
        packet.put("Motor Pos R", motorUtility.getEncoderValue(Motors.SLIDE_RIGHT));
        packet.put("Motor Pos L", motorUtility.getEncoderValue(Motors.SLIDE_LEFT));
        packet.put("Target Velo", motionState.getV());

        if(prevKP != kP || prevKI != kI || prevKD != kD || prevKV != kV || prevKA != kA || prevKStatic != kStatic) {
            coeffs = new PIDCoefficients(kP, kI, kD);
            controller = new PIDFController(coeffs, kV, kA, kStatic);
//            controller = new PIDFController(coeffs);
            prevKP = kP;
            prevKI = kI;
            prevKD = kD;
            prevKV = kV;
            prevKA = kA;
            prevKStatic = kStatic;
        }
    }

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 200 : setpoint, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? setpoint : 200, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, targetVelo, targetAccel);
    }
}