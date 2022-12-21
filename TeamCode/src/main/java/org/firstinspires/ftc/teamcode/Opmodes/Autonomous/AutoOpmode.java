package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.TrackType;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


public class AutoOpmode extends RobotHardware {

    AllianceColor robotColor;
    StartPosition robotStartPos;
    private Executive.RobotStateMachineContextInterface robotStateContext;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    public AprilTagDetection tagOfInterest = null;
    boolean visionInitialized = false;

    @Autonomous(name="Red Left", group="A")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.AUDIENCE;
            RobotStateContext.blue = false;
            super.init();
        }
    }

    @Autonomous(name="Official Blue Left", group="A")
    public static class AutoRedBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.AUDIENCE;
            RobotStateContext.blue = true;
            super.init();
        }
    }

    @Autonomous(name="Blue Left", group="B")
    @Disabled
    public static class AutoBluePickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.FAR;
            super.init();
        }
    }

    @Autonomous(name="Blue Right", group="B")
    @Disabled
    public static class AutoBlueBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.AUDIENCE;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        robotStateContext = new RobotStateContext(this, robotColor, robotStartPos);
        new Thread(this::loadVision).start();
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
        if(!visionInitialized) {
            visionDetection.getLeftPipeline().setTrackType(TrackType.SLEEVE);
            visionInitialized = true;
        }

        ArrayList<AprilTagDetection> currentDetections = visionDetection.getLeftPipeline().getLatestDetections();
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

            if(tagFound)
            {
                telemetry.addData("Tag: ", tagOfInterest.id);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before");
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before");
            }

        }

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