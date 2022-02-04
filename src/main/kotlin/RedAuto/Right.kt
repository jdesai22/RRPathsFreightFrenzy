package RedAuto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import toRadians

class Right {
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    private val combinedConstraints = MecanumConstraints(driveConstraints, Companion.trackWidth)

    private val startPose = Pose2d(-62.5, -10.0, -90.0.toRadians)

    private val depositPose = Pose2d(-25.0, -8.0, -90.0.toRadians)

//    private val storagePark1 = Pose2d(0.0, -8.0, -90.0.toRadians)
//
//    private val storageParkFinal = Pose2d(-36.0, 62.5, -90.0.toRadians)

    private val mineralPark1 = Pose2d(-40.0, -10.0, -90.0.toRadians)

    private val mineralFinalPark = Pose2d(-40.0, -50.0, -90.0.toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)

        builder1.lineToSplineHeading(depositPose, combinedConstraints);

//        val builder2 = TrajectoryBuilder(depositPose, depositPose.heading, combinedConstraints)
//
//        builder2.lineToSplineHeading(storagePark1, combinedConstraints);
//
//        val builder3 = TrajectoryBuilder(storagePark1, storagePark1.heading, combinedConstraints)
//
//        // add a reverse = true here
//        builder3.splineTo(Vector2d(storageParkFinal.x, storageParkFinal.y), storageParkFinal.heading);

        val builder4 = TrajectoryBuilder(depositPose, depositPose.heading, combinedConstraints);

        builder4.lineToSplineHeading(mineralPark1, combinedConstraints);

        val builder5 = TrajectoryBuilder(mineralPark1, mineralPark1.heading, combinedConstraints)

        builder5.forward(40.0)


        // Small Example Routine
//        builder1
//            .splineTo(Vector2d(10.0, 10.0), 0.0)
//            .splineTo(Vector2d(15.0, 15.0), 90.0);

        list.add(builder1.build())
//        list.add(builder2.build())
//        list.add(builder3.build())
        list.add(builder4.build())
        list.add(builder5.build())

        return list
    }

    companion object {
        // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
        private const val trackWidth = 16.0
    }
}

