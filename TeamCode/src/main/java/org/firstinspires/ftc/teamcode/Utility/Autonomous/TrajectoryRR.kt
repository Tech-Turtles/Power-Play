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
    // Start positions (Blue side)
    val startAudience = Pose2d(-60.0,31.75,0.0.toRadians)
    val startFar = Pose2d(-62.5,-31.75,0.0.toRadians)


    // Sleeve trajectories, retrieved through the getSleeveTrajectory() function.
    private var trajectoryStartToSignalParkFar: Trajectory? = null
    private var trajectoryStartToSignalParkMiddle: Trajectory? = null
    private var trajectoryStartToSignalParkAudience: Trajectory? = null

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

        this.trajectoryStartToSignalParkAudience =
            trajectoryBuilder(startAudience, 120.0)
                .splineToConstantHeading(Vector2d(-56.0,55.0), (0.0).toRadians)
                .splineToConstantHeading(Vector2d(-24.0,55.0), 0.0.toRadians)
                .build()

        this.trajectoryStartToSignalParkMiddle =
                trajectoryBuilder(startAudience, 0.0)
                    .splineToConstantHeading(Vector2d(-26.0,35.0), (0.0).toRadians)
                    .build()

        this.trajectoryStartToSignalParkFar =
            trajectoryBuilder(startAudience, 200.0)
                .splineToConstantHeading(Vector2d(-56.0,12.0), (0.0).toRadians)
                .splineToConstantHeading(Vector2d(-26.0,12.0), 0.0.toRadians)
                .build()
    }

    fun getSleeveTrajectory(allianceColor: AllianceColor, signal: Signal): Trajectory? {
        val x = when (signal) {
            Signal.GREEN -> (trajectoryStartToSignalParkAudience)
            Signal.BLUE -> (trajectoryStartToSignalParkMiddle)
            Signal.YELLOW -> (trajectoryStartToSignalParkFar)
            else -> (trajectoryStartToSignalParkMiddle)
        }

        return x
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

val Int.toRadians get() = (Math.toRadians(this.toDouble()))

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