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
    // Start positions (Red side)
    var startAudience = Pose2d(-35.0,-61.75,90.0.toRadians)
    var startFar = Pose2d(35.25,-61.75,0.0.toRadians)


    // Sleeve trajectories, retrieved through the getSleeveTrajectory() function.
    private var trajectoryStartToSignalParkFar: Trajectory? = null
    private var trajectoryStartToSignalParkMiddle: Trajectory? = null
    private var trajectoryStartToSignalParkAudience: Trajectory? = null
    private var trajectoryHighPoleToSignalParkFar: Trajectory? = null
    private var trajectoryHighPoleToSignalParkMiddle: Trajectory? = null
    private var trajectoryHighPoleToSignalParkAudience: Trajectory? = null
    var trajectoryStartToHighPole: Trajectory? = null
    var trajectoryHighPoleToStack: Trajectory? = null
    var trajectoryStackToHighPole: Trajectory? = null
    var trajectorySlowStackGrab: Trajectory? = null

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
        when(color) {
            AllianceColor.BLUE -> {

                startAudience = Pose2d(-35.0,61.75, 270.0.toRadians)
                startFar = Pose2d(35.25,61.75, 270.0.toRadians)

                val startToHighPole = trajectoryBuilder(startAudience, 270.0.toRadians)
                    .splineToConstantHeading(Vector2d(-35.0, 17.0), 270.0.toRadians)
                    .splineToSplineHeading(Pose2d(-35.0, 13.0, 0.0.toRadians), 270.0.toRadians)
                    .build()
                this.trajectoryStartToHighPole = startToHighPole

                val highPoleToStack = trajectoryBuilder(startToHighPole.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-45.0, 13.0), 180.0.toRadians)
                    .build()
                this.trajectoryHighPoleToStack = highPoleToStack

                val slowStackGrab = trajectoryBuilder(highPoleToStack.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-54.0, 13.0), 180.0.toRadians)
                    .build()
                this.trajectorySlowStackGrab = slowStackGrab

                val stackToHighPole = trajectoryBuilder(slowStackGrab.end(),0.0.toRadians)
                    .splineToConstantHeading(Vector2d(-35.0, 13.0), 0.0.toRadians)
                    .build()
                this.trajectoryStackToHighPole = stackToHighPole

                this.trajectoryHighPoleToSignalParkAudience =
                    trajectoryBuilder(stackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,34.5), 180.0.toRadians)
                        .build()

                this.trajectoryHighPoleToSignalParkMiddle =
                    trajectoryBuilder(stackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), (90.0).toRadians)
                        .build()

                this.trajectoryHighPoleToSignalParkFar =
                    trajectoryBuilder(stackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,34.5), 0.0.toRadians)
                        .build()

                this.trajectoryStartToSignalParkAudience =
                    trajectoryBuilder(stackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,34.5), 180.0.toRadians)
                        .build()

                this.trajectoryStartToSignalParkMiddle =
                    trajectoryBuilder(stackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), (90.0).toRadians)
                        .build()

                this.trajectoryStartToSignalParkFar =
                    trajectoryBuilder(stackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,30.0), 0.0.toRadians)
                        .build()
            } else -> {
                this.trajectoryStartToSignalParkAudience =
                    trajectoryBuilder(startAudience, 180.0.toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,-58.5), (90.0).toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,-30.0), 90.0.toRadians)
                        .build()

                this.trajectoryStartToSignalParkMiddle =
                    trajectoryBuilder(startAudience, 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,-30.0), (90.0).toRadians)
                        .build()

                this.trajectoryStartToSignalParkFar =
                    trajectoryBuilder(startAudience, 0.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,-58.5), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,-30.0), 90.0.toRadians)
                        .build()
            
                // High Pole Autonomous Trajectories

                val startToHighPole = trajectoryBuilder(startAudience, 90.0.toRadians)
                    .splineToConstantHeading(Vector2d(-35.0, -17.0), 90.0.toRadians)
                    .splineToSplineHeading(Pose2d(-35.0, -13.0, 0.0.toRadians), 90.0.toRadians)
                    .build()
                this.trajectoryStartToHighPole = startToHighPole

                val highPoleToStack = trajectoryBuilder(startToHighPole.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-45.0, -13.0), 180.0.toRadians)
                    .build()
                this.trajectoryHighPoleToStack = highPoleToStack

                val slowStackGrab = trajectoryBuilder(highPoleToStack.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-53.5, -13.0), 180.0.toRadians, slowVelocityConstraint, slowAccelerationConstraint)
                    .build()
                this.trajectorySlowStackGrab = slowStackGrab

                val stackToHighPole = trajectoryBuilder(slowStackGrab.end(),0.0.toRadians)
                    .splineToConstantHeading(Vector2d(-35.0, -13.0), 0.0.toRadians)
                    .build()
                this.trajectoryStackToHighPole = stackToHighPole

                this.trajectoryHighPoleToSignalParkAudience =
                    trajectoryBuilder(stackToHighPole.end(), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,-30.0), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,-33.0), 180.0.toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,-25.5), 180.0.toRadians)
                        .build()
    
                this.trajectoryHighPoleToSignalParkMiddle =
                    trajectoryBuilder(stackToHighPole.end(), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,-30.0), (270.0).toRadians)
                        .build()
    
                this.trajectoryHighPoleToSignalParkFar =
                    trajectoryBuilder(stackToHighPole.end(), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,-30.0), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,-33.0), 0.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,-25.5), 0.0.toRadians)
                        .build()
            }
        }
    }

    fun getSleeveTrajectory(allianceColor: AllianceColor, signal: Signal): Trajectory? {
        return when (signal) {
            Signal.LEFT -> if (allianceColor == AllianceColor.BLUE) (trajectoryStartToSignalParkFar) else (trajectoryStartToSignalParkAudience)
            Signal.MIDDLE -> (trajectoryStartToSignalParkMiddle)
            Signal.RIGHT -> if (allianceColor == AllianceColor.BLUE) (trajectoryStartToSignalParkAudience) else (trajectoryStartToSignalParkFar)
            else -> (trajectoryStartToSignalParkMiddle)
        }
    }

    fun getHighPoleSleeveTrajectory(allianceColor: AllianceColor, signal: Signal): Trajectory? {
        return when (signal) {
            Signal.LEFT -> if (allianceColor == AllianceColor.BLUE) (trajectoryHighPoleToSignalParkFar) else (trajectoryHighPoleToSignalParkAudience)
            Signal.MIDDLE -> (trajectoryHighPoleToSignalParkMiddle)
            Signal.RIGHT -> if (allianceColor == AllianceColor.BLUE) (trajectoryHighPoleToSignalParkAudience) else (trajectoryHighPoleToSignalParkFar)
            else -> (trajectoryHighPoleToSignalParkMiddle)
        }
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