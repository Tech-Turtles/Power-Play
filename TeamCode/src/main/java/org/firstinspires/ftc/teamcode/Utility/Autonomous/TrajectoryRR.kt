package org.firstinspires.ftc.teamcode.Utility.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive

class TrajectoryRR constructor(sampleMecanumDrive: SampleMecanumDrive){
    val drive: SampleMecanumDrive = sampleMecanumDrive

    private val audienceStart = Pose2d(-62.5,31.75,90.0.toRadians)

    var trajectoryStartToSignalParkLeft: Trajectory? = null
    var trajectoryStartToSignalParkMiddle: Trajectory? = null
    var trajectoryStartToSignalParkRight: Trajectory? = null

    private var velocityConstraint: TrajectoryVelocityConstraint? = null
    private var accelerationConstraint: TrajectoryAccelerationConstraint? = null
    private var slowVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var slowAccelerationConstraint: TrajectoryAccelerationConstraint? = null

    private val slowVelocity: Double = 18.0
    private val slowAcceleration: Double = 40.0

    val list = ArrayList<Trajectory>()

    init {
        velocityConstraint = getMinVelocityConstraint(DriveConstants.MAX_VEL)
        accelerationConstraint = getMinAccelerationConstraint(DriveConstants.MAX_ACCEL)

        slowVelocityConstraint = getMinVelocityConstraint(slowVelocity)
        slowAccelerationConstraint = getMinAccelerationConstraint(slowAcceleration)

        buildTrajectories(AllianceColor.NONE)
    }

    fun resetTrajectories(color: AllianceColor) {
        buildTrajectories(color)
    }

    private fun buildTrajectories(color: AllianceColor) {

        // Example
//        val tempTraj: Trajectory =
//                trajectoryBuilder(STARTPOS, (90.0 - 20.0).toRadians)
//                        .splineToConstantHeading(CENTER.vec(), (20.0 + 90.0).toRadians)
//                        .build()
//        this.mainTraj = tempTraj

        val startToSignalParkLeft: Trajectory =
                trajectoryBuilder(audienceStart, 0.0)
                    .splineToSplineHeading(Pose2d(-34.0,36.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                    .splineToSplineHeading(Pose2d(-10.0,36.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                    .build()
        this.trajectoryStartToSignalParkLeft = startToSignalParkLeft

        val startToSignalParkMiddle: Trajectory =
                trajectoryBuilder(audienceStart, 0.0)
                        .splineToSplineHeading(Pose2d(-34.0,36.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                        .build()
        this.trajectoryStartToSignalParkMiddle = startToSignalParkMiddle

        val startToSignalParkRight: Trajectory =
                trajectoryBuilder(audienceStart, 0.0)
                        .splineToSplineHeading(Pose2d(-34.0,36.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                        .splineToSplineHeading(Pose2d(-60.0,36.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                        .build()
        this.trajectoryStartToSignalParkRight = startToSignalParkRight

    }

    fun toVector2d(pose: Pose2d): Vector2d {
        return Vector2d(pose.x, pose.y)
    }

    private fun  trajectoryBuilder(pose: Pose2d, heading: Double): TrajectoryBuilder{
        return drive.trajectoryBuilder(pose, heading)
    }

    fun  trajectoryBuilder(pose: Pose2d, reversed: Boolean): TrajectoryBuilder{
        return drive.trajectoryBuilder(pose, reversed)
    }

    private fun getMinVelocityConstraint(MaxVelocity: Double): MinVelocityConstraint {
        return MinVelocityConstraint(listOf(
                AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                MecanumVelocityConstraint(MaxVelocity, DriveConstants.TRACK_WIDTH)
        ))
    }

    private fun getMinAccelerationConstraint(MaxAccel: Double): ProfileAccelerationConstraint {
        return ProfileAccelerationConstraint(MaxAccel)
    }
}

val Double.toRadians get() = (Math.toRadians(this))

/*
    +x is the 'positive' direction, and rotation is counter-clockwise around (0,0)
    https://en.wikipedia.org/wiki/Rotation_matrix
 */
fun Pose2d.rotateFrame(rotationRadians: Double): Pose2d
{
    return Pose2d(this.x * kotlin.math.cos(rotationRadians) - this.y * kotlin.math.sin(rotationRadians),
            this.x * kotlin.math.sin(rotationRadians) + this.y * kotlin.math.cos(rotationRadians),
            this.heading + rotationRadians)
}