package org.firstinspires.ftc.teamcode.Utility.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive

class TrajectoryRR constructor(sampleMecanumDrive: SampleMecanumDrive){
    val drive: SampleMecanumDrive = sampleMecanumDrive
    // Start positions (Red side)
    var startAudience = Pose2d(-35.0,-61.75,90.0.toRadians)
    var startFar = Pose2d(35.25,-61.75,90.0.toRadians)


    // Sleeve trajectories, retrieved through the getSleeveTrajectory() function.
    private var trajectoryAudienceStartToSignalParkFar: Trajectory? = null
    private var trajectoryAudienceStartToSignalParkMiddle: Trajectory? = null
    private var trajectoryAudienceStartToSignalParkAudience: Trajectory? = null
    private var trajectoryFarStartToSignalParkFar: Trajectory? = null
    private var trajectoryFarStartToSignalParkMiddle: Trajectory? = null
    private var trajectoryFarStartToSignalParkAudience: Trajectory? = null

    private var trajectoryAudienceHighPoleToSignalParkFar: Trajectory? = null
    private var trajectoryAudienceHighPoleToSignalParkMiddle: Trajectory? = null
    private var trajectoryAudienceHighPoleToSignalParkAudience: Trajectory? = null
    private var trajectoryFarHighPoleToSignalParkFar: Trajectory? = null
    private var trajectoryFarHighPoleToSignalParkMiddle: Trajectory? = null
    private var trajectoryFarHighPoleToSignalParkAudience: Trajectory? = null
    var trajectoryAudienceStartToHighPole: Trajectory? = null
    var trajectoryAudienceHighPoleToStack: Trajectory? = null
    var trajectoryAudienceStackToHighPole: Trajectory? = null
    var trajectoryAudienceSlowStackGrab: Trajectory? = null
    var trajectoryFarStartToHighPole: Trajectory? = null
    var trajectoryFarHighPoleToStack: Trajectory? = null
    var trajectoryFarStackToHighPole: Trajectory? = null
    var trajectoryFarSlowStackGrab: Trajectory? = null

    private var velocityConstraint: TrajectoryVelocityConstraint? = null
    private var accelerationConstraint: TrajectoryAccelerationConstraint? = null
    private var slowVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var slowAccelerationConstraint: TrajectoryAccelerationConstraint? = null

    private val slowVelocity: Double = 50.0
    private val slowAcceleration: Double = 30.0

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
                startFar = Pose2d(35.0,61.75, 270.0.toRadians)

                val audienceStartToHighPole = trajectoryBuilder(startAudience, 270.0.toRadians)
                    .splineToConstantHeading(Vector2d(-35.0, 17.0), 270.0.toRadians)
                    .splineToSplineHeading(Pose2d(-35.0, 13.0, 0.0.toRadians), 270.0.toRadians)
                    .build()
                this.trajectoryAudienceStartToHighPole = audienceStartToHighPole

                val audienceHighPoleToStack = trajectoryBuilder(audienceStartToHighPole.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-45.0, 13.0), 180.0.toRadians)
                    .build()
                this.trajectoryAudienceHighPoleToStack = audienceHighPoleToStack

                val audienceSlowStackGrab = trajectoryBuilder(audienceHighPoleToStack.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-53.25, 13.0), 180.0.toRadians)
                    .build()
                this.trajectoryAudienceSlowStackGrab = audienceSlowStackGrab

                val audienceStackToHighPole = trajectoryBuilder(audienceSlowStackGrab.end(),0.0.toRadians)
                    .splineToConstantHeading(Vector2d(-38.0, 13.0), 0.0.toRadians)
                    .build()
                this.trajectoryAudienceStackToHighPole = audienceStackToHighPole

                this.trajectoryAudienceHighPoleToSignalParkFar =
                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
                        .lineToConstantHeading(Vector2d(-11.5,13.0))
                        .build()

                this.trajectoryAudienceHighPoleToSignalParkMiddle =
                    trajectoryBuilder(audienceStackToHighPole.end(), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,20.0), (90.0).toRadians)
                        .build()

                this.trajectoryAudienceHighPoleToSignalParkAudience =
                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
                        .lineToConstantHeading(Vector2d(-59.0,13.0))
                        .build()

//                this.trajectoryAudienceHighPoleToSignalParkAudience =
//                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
//                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
//                        .splineToConstantHeading(Vector2d(-59.5,34.5), 180.0.toRadians)
//                        .build()
//
//                this.trajectoryAudienceHighPoleToSignalParkMiddle =
//                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
//                        .splineToConstantHeading(Vector2d(-35.0,30.0), (90.0).toRadians)
//                        .build()
//
//                this.trajectoryAudienceHighPoleToSignalParkFar =
//                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
//                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
//                        .splineToConstantHeading(Vector2d(-11.5,34.5), 0.0.toRadians)
//                        .build()

                this.trajectoryAudienceStartToSignalParkAudience =
                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,34.5), 180.0.toRadians)
                        .build()

                this.trajectoryAudienceStartToSignalParkMiddle =
                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), (90.0).toRadians)
                        .build()

                this.trajectoryAudienceStartToSignalParkFar =
                    trajectoryBuilder(audienceStackToHighPole.end(), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,30.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,34.5), 0.0.toRadians)
                        .build()

                val farStartToHighPole = trajectoryBuilder(startFar, 270.0.toRadians)
                    .splineToConstantHeading(Vector2d(35.0, 17.0), 270.0.toRadians)
                    .splineToSplineHeading(Pose2d(35.0, 13.0, 180.0.toRadians), 270.0.toRadians)
                    .build()
                this.trajectoryFarStartToHighPole = farStartToHighPole

                val farHighPoleToStack = trajectoryBuilder(farStartToHighPole.end(), 0.0.toRadians)
                    .splineToConstantHeading(Vector2d(45.0, 13.0), 0.0.toRadians)
                    .build()
                this.trajectoryFarHighPoleToStack = farHighPoleToStack

                val farSlowStackGrab = trajectoryBuilder(farHighPoleToStack.end(), 0.0.toRadians)
                    .splineToConstantHeading(Vector2d(53.5, 13.0), 0.0.toRadians)
                    .build()
                this.trajectoryFarSlowStackGrab = farSlowStackGrab

                val farStackToHighPole = trajectoryBuilder(farSlowStackGrab.end(),180.0.toRadians)
                    .splineToConstantHeading(Vector2d(35.0, 13.0), 180.0.toRadians)
                    .build()
                this.trajectoryFarStackToHighPole = farStackToHighPole

                this.trajectoryFarHighPoleToSignalParkFar =
                    trajectoryBuilder(farStackToHighPole.end(), 90.0.toRadians)
                        .lineToConstantHeading(Vector2d(59.0,13.0))
                        .build()

                this.trajectoryFarHighPoleToSignalParkMiddle =
                    trajectoryBuilder(farStackToHighPole.end(), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,20.0), (90.0).toRadians)
                        .build()

                this.trajectoryFarHighPoleToSignalParkAudience =
                    trajectoryBuilder(farStackToHighPole.end(), 90.0.toRadians)
                        .lineToConstantHeading(Vector2d(11.5,13.0))
                        .build()

                this.trajectoryFarStartToSignalParkAudience =
                    trajectoryBuilder(startFar, 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,40.0), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(59.5,34.5), 0.0.toRadians)
                        .build()

                this.trajectoryFarStartToSignalParkMiddle =
                    trajectoryBuilder(startFar, 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,30.0), (270.0).toRadians)
                        .build()

                this.trajectoryFarStartToSignalParkFar =
                    trajectoryBuilder(startFar, 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,40.0), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(11.5,34.5), 180.0.toRadians)
                        .build()
            } else -> {
                startAudience = Pose2d(-35.0,-61.75,90.0.toRadians)
                startFar = Pose2d(35.0,-61.75,90.0.toRadians)

                this.trajectoryAudienceStartToSignalParkAudience =
                    trajectoryBuilder(startAudience, 180.0.toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,-58.5), (90.0).toRadians)
                        .splineToConstantHeading(Vector2d(-59.5,-30.0), 90.0.toRadians)
                        .build()

                this.trajectoryAudienceStartToSignalParkMiddle =
                    trajectoryBuilder(startAudience, 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,-30.0), (90.0).toRadians)
                        .build()

                this.trajectoryAudienceStartToSignalParkFar =
                    trajectoryBuilder(startAudience, 0.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,-58.5), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(-11.5,-30.0), 90.0.toRadians)
                        .build()
            
                // High Pole Autonomous Trajectories

                val audienceStartToHighPole = trajectoryBuilder(startAudience, 90.0.toRadians)
                    .splineToConstantHeading(Vector2d(-35.0, -17.0), 90.0.toRadians)
                    .splineToSplineHeading(Pose2d(-35.0, -13.0, 0.0.toRadians), 90.0.toRadians)
                    .build()
                this.trajectoryAudienceStartToHighPole = audienceStartToHighPole

                val audienceHighPoleToStack = trajectoryBuilder(audienceStartToHighPole.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-45.0, -13.0), 180.0.toRadians)
                    .build()
                this.trajectoryAudienceHighPoleToStack = audienceHighPoleToStack

                val audienceSlowStackGrab = trajectoryBuilder(audienceHighPoleToStack.end(), 180.0.toRadians)
                    .splineToConstantHeading(Vector2d(-52.25, -13.0), 180.0.toRadians)
                    .build()
                this.trajectoryAudienceSlowStackGrab = audienceSlowStackGrab

                val audienceStackToHighPole = trajectoryBuilder(audienceSlowStackGrab.end(),0.0.toRadians)
                    .splineToConstantHeading(Vector2d(-38.0, -13.0), 0.0.toRadians)
                    .build()
                this.trajectoryAudienceStackToHighPole = audienceStackToHighPole

                this.trajectoryAudienceHighPoleToSignalParkAudience =
                    trajectoryBuilder(audienceStackToHighPole.end(), 270.0.toRadians)
                        .lineToConstantHeading(Vector2d(-59.0,-13.0))
                        .build()

                this.trajectoryAudienceHighPoleToSignalParkMiddle =
                    trajectoryBuilder(audienceStackToHighPole.end(), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(-35.0,-20.0), (90.0).toRadians)
                        .build()

                this.trajectoryAudienceHighPoleToSignalParkFar =
                    trajectoryBuilder(audienceStackToHighPole.end(), 270.0.toRadians)
                        .lineToConstantHeading(Vector2d(-11.5,-13.0))
                        .build()
            
                // Far trajectories

                val farStartToHighPole = trajectoryBuilder(startFar, 90.0.toRadians)
                    .splineToConstantHeading(Vector2d(35.0, -17.0), 90.0.toRadians)
                    .splineToSplineHeading(Pose2d(35.0, -13.0, 180.0.toRadians), 90.0.toRadians)
                    .build()
                this.trajectoryFarStartToHighPole = farStartToHighPole
        
                val farHighPoleToStack = trajectoryBuilder(farStartToHighPole.end(), 0.0.toRadians)
                    .splineToConstantHeading(Vector2d(45.0, -13.0), 0.0.toRadians)
                    .build()
                this.trajectoryFarHighPoleToStack = farHighPoleToStack
        
                val farSlowStackGrab = trajectoryBuilder(farHighPoleToStack.end(), 0.0.toRadians)
                    .splineToConstantHeading(Vector2d(53.5, -13.0), 0.0.toRadians)
                    .build()
                this.trajectoryFarSlowStackGrab = farSlowStackGrab
        
                val farStackToHighPole = trajectoryBuilder(farSlowStackGrab.end(),180.0.toRadians)
                    .splineToConstantHeading(Vector2d(35.0, -13.0), 180.0.toRadians)
                    .build()
                this.trajectoryFarStackToHighPole = farStackToHighPole

                this.trajectoryFarHighPoleToSignalParkFar =
                    trajectoryBuilder(farStackToHighPole.end(), 270.0.toRadians)
                        .lineToConstantHeading(Vector2d(59.0,-13.0))
                        .build()

                this.trajectoryFarHighPoleToSignalParkMiddle =
                    trajectoryBuilder(farStackToHighPole.end(), 270.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,-20.0), (90.0).toRadians)
                        .build()

                this.trajectoryFarHighPoleToSignalParkAudience =
                    trajectoryBuilder(farStackToHighPole.end(), 270.0.toRadians)
                        .lineToConstantHeading(Vector2d(11.5,-13.0))
                        .build()
        
                this.trajectoryFarStartToSignalParkAudience =
                    trajectoryBuilder(startFar, 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,-40.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(59.5,-34.5), 0.0.toRadians)
                        .build()
        
                this.trajectoryFarStartToSignalParkMiddle =
                    trajectoryBuilder(startFar, 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,-30.0), (90.0).toRadians)
                        .build()
        
                this.trajectoryFarStartToSignalParkFar =
                    trajectoryBuilder(startFar, 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(35.0,-40.0), 90.0.toRadians)
                        .splineToConstantHeading(Vector2d(11.5,-34.5), 180.0.toRadians)
                        .build()
            }
        }
    }

    fun getSleeveTrajectory(allianceColor: AllianceColor, signal: Signal): Trajectory? {
        return when (signal) {
            Signal.LEFT -> if (allianceColor == AllianceColor.BLUE) (trajectoryAudienceStartToSignalParkFar) else (trajectoryAudienceStartToSignalParkAudience)
            Signal.MIDDLE -> (trajectoryAudienceStartToSignalParkMiddle)
            Signal.RIGHT -> if (allianceColor == AllianceColor.BLUE) (trajectoryAudienceStartToSignalParkAudience) else (trajectoryAudienceStartToSignalParkFar)
            else -> (trajectoryAudienceStartToSignalParkMiddle)
        }
    }

    fun getAudienceHighPoleSleeveTrajectory(allianceColor: AllianceColor, signal: Signal): Trajectory? {
        return when (signal) {
            Signal.LEFT -> if (allianceColor == AllianceColor.BLUE) (trajectoryAudienceHighPoleToSignalParkFar) else (trajectoryAudienceHighPoleToSignalParkAudience)
            Signal.MIDDLE -> (trajectoryAudienceHighPoleToSignalParkMiddle)
            Signal.RIGHT -> if (allianceColor == AllianceColor.BLUE) (trajectoryAudienceHighPoleToSignalParkAudience) else (trajectoryAudienceHighPoleToSignalParkFar)
            else -> (trajectoryAudienceHighPoleToSignalParkMiddle)
        }
    }

    fun getFarHighPoleSleeveTrajectory(allianceColor: AllianceColor, signal: Signal): Trajectory? {
        return when (signal) {
            Signal.LEFT -> if (allianceColor == AllianceColor.BLUE) (trajectoryFarHighPoleToSignalParkFar) else (trajectoryFarHighPoleToSignalParkAudience)
            Signal.MIDDLE -> (trajectoryFarHighPoleToSignalParkMiddle)
            Signal.RIGHT -> if (allianceColor == AllianceColor.BLUE) (trajectoryFarHighPoleToSignalParkAudience) else (trajectoryFarHighPoleToSignalParkFar)
            else -> (trajectoryFarHighPoleToSignalParkMiddle)
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