//package org.firstinspires.ftc.teamcode.Utility.Autonomous
//
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.acmerobotics.roadrunner.trajectory.Trajectory
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
//import com.acmerobotics.roadrunner.trajectory.constraints.*
//import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants
//import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive
//import kotlin.math.abs
//import kotlin.math.pow
//import kotlin.math.sqrt
//
//class TrajectoryRR constructor(sampleMecanumDrive: SampleMecanumDrive){
//    val drive: SampleMecanumDrive = sampleMecanumDrive
//
//    var startWarehouse = Pose2d(9.5, 61.75, 90.0.toRadians)
//    var startCarousel = Pose2d(-27.875, 61.75, (90.0).toRadians)
//    private var shippingHubAlign = Pose2d(-12.0, 48.625, (90.0).toRadians)
//    private var shippingHub = Pose2d(-12.0, 41.375, (90.0).toRadians)
//    private var verticalBarrierAlign = Pose2d(6.5, 65.0, 0.0.toRadians)
//    private var carouselAlign = Pose2d(-52.0,58.0,(270.0).toRadians)
//    private var carousel = Pose2d(-57.0,58.0,(270.0).toRadians)
//    private var depotPark = Pose2d(-61.0,36.5,(0.0).toRadians)
//
//    var trajectoryCarouselStartToHub: Trajectory? = null
//    var trajectoryWarehouseStartToHub: Trajectory? = null
//    var trajectoryHubToCarousel: Trajectory? = null
//    var trajectoryHubToDepotPark: Trajectory? = null
//    var trajectoryCarouselToDepot: Trajectory? = null
//    var trajectoryStartToCarousel: Trajectory? = null
//    var trajectoryCarouselToHub: Trajectory? = null
//    var trajectoryHubToVerticalBarrier: Trajectory? = null
//    var trajectoryVerticalBarrierToWarehouse: Trajectory? = null
//    var trajectoryWarehouseThroughVerticalBarrier: Trajectory? = null
//    var trajectoryVerticalBarrierToHubAlign: Trajectory? = null
//    var trajectorySecondCargoHubAlignToDrop: Trajectory? = null
//    var trajectoryHubToWarehousePark: Trajectory? = null
//    var trajectoryWarehouseParkToSharedAlign: Trajectory? = null
//
//    private var velocityConstraint: TrajectoryVelocityConstraint? = null
//    private var accelerationConstraint: TrajectoryAccelerationConstraint? = null
//    private var slowVelocityConstraint: TrajectoryVelocityConstraint? = null
//    private var slowAccelerationConstraint: TrajectoryAccelerationConstraint? = null
//    private var ringVelocityConstraint: TrajectoryVelocityConstraint? = null
//    private var ringAccelerationConstraint: TrajectoryAccelerationConstraint? = null
//    private var superVelocityConstraint: TrajectoryVelocityConstraint? = null
//    private var superAccelerationConstraint: TrajectoryAccelerationConstraint? = null
//
//    private val slowVelocity: Double = 18.0
//    private val slowAcceleration: Double = 40.0
//    private val ringVelocity: Double = 60.0
//    private val ringAcceleration: Double = 40.0
//    private val superVelocity: Double = 75.0
//    private val superAcceleration: Double = 40.0
//
//    val list = ArrayList<Trajectory>()
//
//    init {
//        velocityConstraint = getMinVelocityConstraint(DriveConstants.MAX_VEL)
//        accelerationConstraint = getMinAccelerationConstraint(DriveConstants.MAX_ACCEL)
//
//        slowVelocityConstraint = getMinVelocityConstraint(slowVelocity)
//        slowAccelerationConstraint = getMinAccelerationConstraint(slowAcceleration)
//
//        ringVelocityConstraint = getMinVelocityConstraint(ringVelocity)
//        ringAccelerationConstraint = getMinAccelerationConstraint(ringAcceleration)
//
//        superVelocityConstraint = getMinVelocityConstraint(superVelocity)
//        superAccelerationConstraint = getMinAccelerationConstraint(superAcceleration)
//
//        buildTrajectories(AllianceColor.NONE)
//    }
//
//    fun resetTrajectories(color: AllianceColor) {
//        buildTrajectories(color)
//    }
//
//    private fun buildTrajectories(color: AllianceColor) {
//
//        // Example
////        val tempTraj: Trajectory =
////                trajectoryBuilder(STARTPOS, (90.0 - 20.0).toRadians)
////                        .splineToConstantHeading(CENTER.vec(), (20.0 + 90.0).toRadians)
////                        .build()
////        this.mainTraj = tempTraj
//        // Dumb way to do different trajectories based on color (not racist I swear)
////        when(color) {
////            AllianceColor.BLUE -> {
////                START_DEPOT = Pose2d(0.0, 70.0 - (ROBOT_WIDTH / 2), 0.0)
////                    START_CAROUSEL = Pose2d(-38.0, 61.0, (-90.0).toRadians)
////                var CAROUSEL_INTER = Pose2d(-46.5, 48.75, (-90.0).toRadians)
////                    CAROUSEL_ALIGN = Pose2d(-61.5, 48.75, (-90.0).toRadians)
////                var CAROUSEL = Pose2d(-61.5, 58.0, (-90.0).toRadians)
////                SHIPPING_HUB_PARK = Pose2d(-61.75, 40.0, 0.0)
////                VERTICAL_BARRIER_ALIGN = Pose2d(0.0, 40 + (ROBOT_WIDTH / 2), 0.0)
////                WAREHOUSE_PARK = Pose2d(30.0 + ROBOT_LENGTH, 37 + (ROBOT_WIDTH / 2), 0.0)
////                val startToPark: Trajectory =
////                        trajectoryBuilder(START_DEPOT, (-90.0).toRadians)
////                                .lineToConstantHeading(VERTICAL_BARRIER_ALIGN.vec())
////                                .splineToConstantHeading(WAREHOUSE_PARK.vec(), 0.0)
////                                .build()
////                this.trajectoryStartToPark = startToPark
////
////                val startToCarousel: Trajectory =
////                        trajectoryBuilder(START_CAROUSEL, (-90.0).toRadians)
////                                .lineToConstantHeading(CAROUSEL_INTER.vec())
////                                .splineToConstantHeading(CAROUSEL_ALIGN.vec(), (-90.0).toRadians)
////                                .splineToConstantHeading(CAROUSEL.vec(), (-90.0).toRadians)
////                                .build()
////                this.trajectoryStartToCarousel = startToCarousel
////                val carouselToHub: Trajectory =
////                        trajectoryBuilder(CAROUSEL, 0.0)
////                                .lineToConstantHeading(SHIPPING_HUB_PARK.vec())
////                                .build()
////                this.trajectoryCarouselToHub = carouselToHub
////            } else -> {
////                val startToPark: Trajectory =
////                        trajectoryBuilder(START_DEPOT, (-90.0).toRadians)
////                                .lineToConstantHeading(VERTICAL_BARRIER_ALIGN.vec())
////                                .splineToConstantHeading(WAREHOUSE_PARK.vec(), 0.0)
////                                .build()
////                this.trajectoryStartToPark = startToPark
////
////                val startToCarousel: Trajectory =
////                        trajectoryBuilder(START_CAROUSEL, (180.0).toRadians)
////                                .lineToConstantHeading(CAROUSEL_ALIGN.vec())
////                                .build()
////                this.trajectoryStartToCarousel = startToCarousel
////                val carouselToHub: Trajectory =
////                        trajectoryBuilder(CAROUSEL_ALIGN, (-90.0).toRadians)
////                                .lineToConstantHeading(SHIPPING_HUB_PARK.vec())
////                                .build()
////                this.trajectoryCarouselToHub = carouselToHub
////            }
////        }
//
//        when(color) {
//            AllianceColor.RED -> {
//
//                startCarousel = Pose2d(-42.5, -61.75, (-90.0).toRadians)
//                startWarehouse = Pose2d(4.5, -62.25, (-90.0).toRadians)
//
//                shippingHubAlign = Pose2d(-12.0, -49.625, (-90.0).toRadians)
//                shippingHub = Pose2d(-12.0, -41.375, (-90.0).toRadians)
//                carouselAlign = Pose2d(-57.5,-50.0,(90.0).toRadians)
//                carousel = Pose2d(-57.75,-56.5,(90.0).toRadians)
//                depotPark = Pose2d(-61.0,-36.5 - 2.0,(0.0).toRadians)
//
//                verticalBarrierAlign = Pose2d(6.5, -65.0, 0.0.toRadians)
//
//                val startToHub: Trajectory = trajectoryBuilder(startCarousel, true)
//                        .splineToConstantHeading(shippingHubAlign.vec(), (90.0).toRadians)
//                        .lineToConstantHeading(shippingHub.vec(), slowVelocityConstraint, slowAccelerationConstraint)
//                        .build()
//                this.trajectoryCarouselStartToHub = startToHub
//
//                val hubToCarousel: Trajectory = trajectoryBuilder(startToHub.end(), true)
//                        .lineToConstantHeading(shippingHubAlign.vec())
//                        .splineToSplineHeading(carouselAlign, 270.0.toRadians)
//                        .splineToConstantHeading(carousel.vec(), 270.0.toRadians)
//                        .build()
//                this.trajectoryHubToCarousel = hubToCarousel
//
//                val carouselToDepotPark: Trajectory = trajectoryBuilder(hubToCarousel.end(), true)
//                        .lineToConstantHeading(depotPark.vec())
//                        .build()
//                this.trajectoryCarouselToDepot = carouselToDepotPark
//
//                val hubToDepotPark: Trajectory = trajectoryBuilder(startToHub.end(), true,)
//                        .lineToConstantHeading(shippingHubAlign.vec())
//                        .splineToConstantHeading(depotPark.vec(), 0.0)
//                        .build()
//                this.trajectoryHubToDepotPark = hubToDepotPark
//
//                val warehouseStartToHub: Trajectory = trajectoryBuilder(startWarehouse, true)
//                        .splineToConstantHeading(shippingHubAlign.vec(), (90.0).toRadians)
//                        .lineToConstantHeading(shippingHub.vec(), slowVelocityConstraint, slowAccelerationConstraint)
//                        .build()
//                this.trajectoryWarehouseStartToHub = warehouseStartToHub
//
//                val hubToVerticalBarrier: Trajectory = trajectoryBuilder(warehouseStartToHub.end(), false)
//                        .lineToConstantHeading(shippingHubAlign.vec())
//                        .splineToSplineHeading(shippingHubAlign.plus(Pose2d(0.1,0.0, (90.0).toRadians)), (30.0).toRadians)
//                        .splineToConstantHeading(verticalBarrierAlign.vec(), 0.0)
//                        .forward(30.0)
//                        .build()
//                this.trajectoryHubToVerticalBarrier = hubToVerticalBarrier
//
//                val verticalBarrierToWarehouse: Trajectory = trajectoryBuilder(hubToVerticalBarrier.end(), false)
//                        .forward(14.0, slowVelocityConstraint, slowAccelerationConstraint)
//                        .build()
//                this.trajectoryVerticalBarrierToWarehouse = verticalBarrierToWarehouse
//
//                val warehousePastVerticalBarrier: Trajectory = trajectoryBuilder(verticalBarrierToWarehouse.end(), false)
//                        .back(50.0)
//                        .build()
//                this.trajectoryWarehouseThroughVerticalBarrier = warehousePastVerticalBarrier
//
//                val verticalBarrierToHubAlign: Trajectory = trajectoryBuilder(warehousePastVerticalBarrier.end(), false)
//                        .strafeLeft(10.0)
//                        .splineToSplineHeading(warehousePastVerticalBarrier.end().plus(Pose2d(0.1,10.0, (-90.0).toRadians)), (-160.0).toRadians)
//                        .build()
//                this.trajectoryVerticalBarrierToHubAlign = verticalBarrierToHubAlign
//
//                val secondCargoHubAlignToDrop: Trajectory = trajectoryBuilder(verticalBarrierToHubAlign.end(), false)
//                        .lineToConstantHeading(shippingHub.vec().plus(Vector2d(3.0, -2.0)))
//                        .build()
//                this.trajectorySecondCargoHubAlignToDrop = secondCargoHubAlignToDrop
//
//                val hubToWarehousePark: Trajectory = trajectoryBuilder(warehouseStartToHub.end(), false)
//                        .lineToConstantHeading(shippingHubAlign.vec())
//                        .splineToSplineHeading(shippingHubAlign.plus(Pose2d(0.1,0.0, (90.0).toRadians)), (30.0).toRadians)
//                        .splineToConstantHeading(verticalBarrierAlign.vec().plus(Vector2d(0.0, -2.0)), 0.0, superVelocityConstraint, superAccelerationConstraint)
//                        .forward(40.0, superVelocityConstraint, superAccelerationConstraint)
//                        .build()
//                this.trajectoryHubToWarehousePark = hubToWarehousePark
//
//                val warehouseParkToSharedAlign: Trajectory = trajectoryBuilder(hubToWarehousePark.end(), false)
//                        .splineToConstantHeading(hubToWarehousePark.end().vec().plus(Vector2d(-8.0, 0.0)), 0.0)
//                        .splineToConstantHeading(hubToWarehousePark.end().vec().plus(Vector2d(-8.0, 20.0)), 0.0)
//                        .splineToConstantHeading(hubToWarehousePark.end().vec().plus(Vector2d(18.0, 20.0)), 0.0)
//                        .splineToSplineHeading(hubToWarehousePark.end().plus(Pose2d(18.001, 20.0, 90.0.toRadians)), 0.0.toRadians)
//                        .build()
//                this.trajectoryWarehouseParkToSharedAlign = warehouseParkToSharedAlign
//            } else -> {
//                val carouselStartToHub: Trajectory = trajectoryBuilder(startCarousel, true)
//                        .splineToConstantHeading(shippingHubAlign.vec(), (-90.0).toRadians)
//                        .lineToConstantHeading(shippingHub.vec(), slowVelocityConstraint, slowAccelerationConstraint)
//                        .build()
//                this.trajectoryCarouselStartToHub = carouselStartToHub
//
//                val hubToCarousel: Trajectory = trajectoryBuilder(carouselStartToHub.end(), false)
//                        .lineToConstantHeading(shippingHubAlign.vec())
//                        .splineTo(shippingHubAlign.vec().plus(Vector2d(1.0,1.0)), 0.0)
//                        .splineToConstantHeading(carouselAlign.vec(), 0.0)
//                        .splineToConstantHeading(carousel.vec(),0.0, slowVelocityConstraint, slowAccelerationConstraint)
//                        .build()
//                this.trajectoryHubToCarousel = hubToCarousel
//
//                val carouselToDepotPark: Trajectory = trajectoryBuilder(hubToCarousel.end(), 0.0)
//                        .lineToConstantHeading(depotPark.vec())
//                        .build()
//                this.trajectoryCarouselToDepot = carouselToDepotPark
//
//                val warehouseStartToHub: Trajectory = trajectoryBuilder(startWarehouse, true)
//                        .splineToConstantHeading(shippingHubAlign.vec(), (-90.0).toRadians)
//                        .lineToConstantHeading(shippingHub.vec(), slowVelocityConstraint, slowAccelerationConstraint)
//                        .build()
//                this.trajectoryWarehouseStartToHub = warehouseStartToHub
//
//                val hubToVerticalBarrier: Trajectory = trajectoryBuilder(warehouseStartToHub.end(), false)
//                        .lineToConstantHeading(shippingHubAlign.vec())
//                        .splineToSplineHeading(shippingHubAlign.plus(Pose2d(0.1,0.0, (-90.0).toRadians)), (-30.0).toRadians)
//                        .splineToConstantHeading(verticalBarrierAlign.vec(), 0.0)
//                        .forward(30.0)
//                        .build()
//                this.trajectoryHubToVerticalBarrier = hubToVerticalBarrier
//
//                val verticalBarrierToWarehouse: Trajectory = trajectoryBuilder(hubToVerticalBarrier.end(), false)
//                        .forward(14.0, slowVelocityConstraint, slowAccelerationConstraint)
//                        .build()
//                this.trajectoryVerticalBarrierToWarehouse = verticalBarrierToWarehouse
//
//                val warehousePastVerticalBarrier: Trajectory = trajectoryBuilder(verticalBarrierToWarehouse.end(), false)
//                        .back(50.0)
//                        .build()
//                this.trajectoryWarehouseThroughVerticalBarrier = warehousePastVerticalBarrier
//
//                val verticalBarrierToHubAlign: Trajectory = trajectoryBuilder(warehousePastVerticalBarrier.end(), false)
//                        .strafeRight(10.0)
//                        .splineToSplineHeading(warehousePastVerticalBarrier.end().plus(Pose2d(0.1,-10.0, (90.0).toRadians)), (160.0).toRadians)
//                        .build()
//                this.trajectoryVerticalBarrierToHubAlign = verticalBarrierToHubAlign
//
//                val secondCargoHubAlignToDrop: Trajectory = trajectoryBuilder(verticalBarrierToHubAlign.end(), false)
//                        .lineToConstantHeading(shippingHub.vec().plus(Vector2d(3.0, 2.0)))
//                        .build()
//                this.trajectorySecondCargoHubAlignToDrop = secondCargoHubAlignToDrop
//
//                val hubToWarehousePark: Trajectory = trajectoryBuilder(warehouseStartToHub.end(), false)
//                        .lineToConstantHeading(shippingHubAlign.vec())
//                        .splineToSplineHeading(shippingHubAlign.plus(Pose2d(0.1,0.0, (-90.0).toRadians)), (-30.0).toRadians)
//                        .splineToConstantHeading(verticalBarrierAlign.vec().plus(Vector2d(0.0, 2.0)), 0.0, superVelocityConstraint, superAccelerationConstraint)
//                        .forward(40.0, superVelocityConstraint, superAccelerationConstraint)
//                        .build()
//                this.trajectoryHubToWarehousePark = hubToWarehousePark
//            }
//        }
//    }
//
//    fun toVector2d(pose: Pose2d): Vector2d {
//        return Vector2d(pose.x, pose.y)
//    }
//
//    private fun  trajectoryBuilder(pose: Pose2d, heading: Double): TrajectoryBuilder{
//        return drive.trajectoryBuilder(pose, heading)
//    }
//
//    fun  trajectoryBuilder(pose: Pose2d, reversed: Boolean): TrajectoryBuilder{
//        return drive.trajectoryBuilder(pose, reversed)
//    }
//
//    companion object {
//        @JvmStatic
//        fun getNearestCornerPose2d(pose: Pose2d): Pose2d {
//            val flipOffset = Pose2d(0.75, 0.0, 180.0.toRadians)
//            val corners = ArrayList<Pose2d>()
//            corners.add(Pose2d(-61.5, -61.25, (0.0).toRadians))
//            // TODO: re-enable other 3 corners
////            corners.add(Pose2d( 60.75, -61.0, (0.0).toRadians))
////            corners.add(Pose2d(-61.5, 61.0, (0.0).toRadians))
////            corners.add(Pose2d( 60.75, 61.0, (0.0).toRadians))
//
//            // Which direction, fwd 0.0 or reverse 180.0?
//            // TODO: re-enable forward orientation
//            val isHeadingFwd = false // HARDCODED
//            //val isHeadingFwd = Math.abs(0.0.toRadians - pose.heading) < 90.0.toRadians
//
//            val orientationOffset =
//                if(isHeadingFwd) Pose2d(0.0, 0.0, 0.0)
//                else flipOffset
//
//            // Which point is the closest?
//            var closestCorner = corners[0].plus(orientationOffset)
//            var closestDistance = getDistance(pose, closestCorner)
//            var newCorner: Pose2d?
//            var newDistance: Double?
//            for(corner in corners) {
//                newCorner = corner
//                newDistance = getDistance(pose, newCorner)
//                if(newDistance < closestDistance) {
//                    closestCorner = newCorner.plus(orientationOffset)
//                    closestDistance = newDistance
//                }
//            }
//            return closestCorner
//        }
//
//        private fun getDistance(pose1: Pose2d, pose2: Pose2d): Double {
//            val deltaX = abs(pose1.x - pose2.x)
//            val deltaY = abs(pose1.y - pose2.y)
//            return sqrt(deltaX.pow(2.0) + deltaY.pow(2.0))
//        }
//    }
//
//    private fun getMinVelocityConstraint(MaxVelocity: Double): MinVelocityConstraint {
//        return MinVelocityConstraint(listOf(
//                AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                MecanumVelocityConstraint(MaxVelocity, DriveConstants.TRACK_WIDTH)
//        ))
//    }
//
//    private fun getMinAccelerationConstraint(MaxAccel: Double): ProfileAccelerationConstraint {
//        return ProfileAccelerationConstraint(MaxAccel)
//    }
//}
//
//val Double.toRadians get() = (Math.toRadians(this))
//
///*
//    +x is the 'positive' direction, and rotation is counter-clockwise around (0,0)
//    https://en.wikipedia.org/wiki/Rotation_matrix
// */
//fun Pose2d.rotateFrame(rotationRadians: Double): Pose2d
//{
//    return Pose2d(this.x * kotlin.math.cos(rotationRadians) - this.y * kotlin.math.sin(rotationRadians),
//            this.x * kotlin.math.sin(rotationRadians) + this.y * kotlin.math.cos(rotationRadians),
//            this.heading + rotationRadians)
//}