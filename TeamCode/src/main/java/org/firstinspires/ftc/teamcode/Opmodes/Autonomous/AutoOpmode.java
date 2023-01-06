package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.CombinedTracker;
import org.firstinspires.ftc.teamcode.Vision.TrackType;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


public class AutoOpmode extends RobotHardware {

    public static AllianceColor robotColor = AllianceColor.RED;
    StartPosition robotStartPos;
    private Executive.RobotStateMachineContextInterface robotStateContext;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    public AprilTagDetection tagOfInterest = null;

    @Autonomous(name="Red Left", group="A")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.AUDIENCE;
            RobotStateContext.far = false;
            super.init();
        }
    }

    @Autonomous(name="Red Right", group="A")
    public static class AutoRedBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            RobotStateContext.far = true;
            robotStartPos = StartPosition.AUDIENCE;
            super.init();
        }
    }

    @Autonomous(name="Blue Left", group="B")
    public static class AutoBluePickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            RobotStateContext.far = true;
            robotStartPos = StartPosition.AUDIENCE;
            super.init();
        }
    }

    @Autonomous(name="Blue Right", group="B")
    public static class AutoBlueBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.AUDIENCE;
            RobotStateContext.far = false;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        CombinedTracker.trackType = TrackType.SLEEVE;
        robotStateContext = new RobotStateContext(this, robotColor, robotStartPos);
        loadVision();
        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        robotStateContext.init();
        telemetry.addData("Initialization: ", "Successful!");
    }

    @Override
    public void init_loop() {
        super.init_loop();
        primary.update();
        if(visionDetection == null || visionDetection.getLeftPipeline() == null)
            return;

        ArrayList<AprilTagDetection> currentDetections = visionDetection.getLeftPipeline().getLatestDetections();
        if(visionDetection.getRightPipeline() != null)
            currentDetections.addAll(visionDetection.getRightPipeline().getLatestDetections());

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(!tagFound)
                telemetry.addLine("Cannot see April Tag");

        } else
            telemetry.addLine("Cannot see April Tag");

        telemetry.addData("Tag", tagOfInterest == null ? "NONE" : tagOfInterest.id);

        servoUtility.setAngle(Servos.LEFT_ARM, Configuration.ServoPosition.START.getLeft());
        servoUtility.setAngle(Servos.RIGHT_ARM, Configuration.ServoPosition.START.getRight());

        servoUtility.setAngle(Servos.CLAW, Configuration.CLAW_CLOSED);

        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        mecanumDrive.update(packet);
        robotStateContext.update();
        if(packet != null) {
            packet.put("Period Average: ", df_precise.format(period.getAveragePeriodSec()) + "s");
            packet.put("Period Max:     ", df_precise.format(period.getMaxPeriodSec()) + "s");
            packet.put("State:          ", robotStateContext.getCurrentState());
        }
    }
}