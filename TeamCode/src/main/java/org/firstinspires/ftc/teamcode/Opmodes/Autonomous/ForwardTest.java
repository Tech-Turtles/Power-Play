package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

@Autonomous(name = "forward test")
public class ForwardTest extends RobotHardware {

    @Override
    public void init() {
        super.init();
        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    @Override
    public void start() {
        super.start();
        Trajectory trajectoryForward = mecanumDrive.trajectoryBuilder(new Pose2d())
                .strafeRight(62.25)
                .build();
        mecanumDrive.followTrajectoryAsync(trajectoryForward);
    }

    @Override
    public void loop() {
        super.loop();
        mecanumDrive.update(packet);
    }
}
