package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
//
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Transform2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.spartronics4915.lib.T265Camera;
//
//import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
//
//@Autonomous(name = "RealsenseTest")
public class RealsenseTest extends RobotHardware {
//
//    public static double precisionMode = 1.0;
//    public static double precisionPercentage = 0.35;
//    public static double linearSpeed = 1.0;
//    public static double lateralSpeed = 1.0;
//    public static double rotationSpeed = 0.8;
//    public static int direction = 1;
//
//    private static T265Camera slamera = null;
//
//    @Override
//    public void init() {
//        super.init();
//        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
//        if (slamera == null) {
//            slamera = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
//        }
//        slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(-27.875, 61.75, new Rotation2d(0)));
//    }
//
//
//    @Override
//    public void init_loop() {
//        super.init_loop();
//    }
//
//    @Override
//    public void start() {
//        super.start();
//        if(!slamera.isStarted())
//            slamera.start();
//    }
//
//    final int robotRadius = 7; // inches
//
//    @Override
//    public void loop() {
//        super.loop();
//        mecanumDrive.update(null);
//        com.acmerobotics.roadrunner.geometry.Pose2d driveDirection;
//        if(primary.AOnce()) {
//            precisionMode = precisionMode == 1 ? precisionPercentage : 1;
//        }
//
//        if(primary.YOnce()) {
//            direction = -direction;
//        }
//
//        driveDirection = new com.acmerobotics.roadrunner.geometry.Pose2d(
//                (-primary.left_stick_y * direction) * linearSpeed * precisionMode,
//                (-primary.left_stick_x * direction) * lateralSpeed * precisionMode,
//                -primary.right_stick_x * rotationSpeed * precisionMode
//        );
//
//        mecanumDrive.setWeightedDrivePower(driveDirection);
//
////        Canvas field = packet.fieldOverlay();
//
//        T265Camera.CameraUpdate up = slamera.getLastReceivedCameraUpdate();
//
//
//        telemetry.addData("Position: ", "%s, %s", up.pose.getTranslation().getX(), up.pose.getTranslation().getY());
//        telemetry.addData("Rotation: ", up.pose.getHeading());
//        telemetry.addData("Period Average: ", df_precise.format(period.getAveragePeriodSec()) + "s");
//        telemetry.addData("Period Max:     ", df_precise.format(period.getMaxPeriodSec()) + "s");
////        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
////        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
////        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
////        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
////        field.strokeLine(x1, y1, x2, y2);
//    }
//
//    @Override
//    public void stop() {
//        super.stop();
//    }
}
