package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.teamcode.auto.ParkingLocation
import org.firstinspires.ftc.teamcode.auto.ParkingLocationAnalyzer
import org.firstinspires.ftc.teamcode.common.commandbase.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.GrabberStateCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand
import org.firstinspires.ftc.teamcode.common.hardware.Robot

@Autonomous(name = "Full Left", group = "auto")
class AutoFullLeft : LinearOpMode() {
    override fun runOpMode() {
        CommandScheduler.getInstance().reset()

        val robot = Robot(hardwareMap, Robot.OpMode.AUTO)
        robot.drive.poseEstimate = Pose2d()

        val parkingLocationAnalyzer = ParkingLocationAnalyzer(hardwareMap)
        var parkingLocation = ParkingLocation.CENTER

        // To break the trajectory spline path we need to pause for a very short amount of time.
        // This prevents the trajectory builder from trying to spline the paths together when we don't want it to.
        val instantTime = 0.00000000001

        val trajectory1 = robot.drive.trajectorySequenceBuilder()
            .splineToConstantHeading(Vector2d(3.0, 3.0), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(44.0, 5.0), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(52.5, -5.0), Math.toRadians(-60.0))
            .build()

        val trajectory2 = robot.drive.trajectorySequenceBuilder(trajectory1.end())
            .lineTo(Vector2d(52.0, 4.0))
            .waitSeconds(instantTime)
            .splineToSplineHeading(Pose2d(55.0, 11.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(55.0, 24.25), Math.toRadians(90.0))
            .build()

        val trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end(), true)
            .splineToConstantHeading(Vector2d(52.5, 5.0), Math.toRadians(-90.0))
            .splineToSplineHeading(Pose2d(53.5, -5.0, Math.toRadians(0.0)), Math.toRadians(-70.0))
            .build()

       val trajectory4 = robot.drive.trajectorySequenceBuilder(trajectory3.end())
            .lineTo(Vector2d(45.0, -3.5))
            .waitSeconds(instantTime)
            .splineToSplineHeading(Pose2d(55.0, 11.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(53.0, 24.25), Math.toRadians(90.0))
            .build()

        val trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end(), true)
            .splineToConstantHeading(Vector2d(54.0, 5.0), Math.toRadians(-90.0))
            .splineToSplineHeading(Pose2d(54.0, -5.0, Math.toRadians(0.0)), Math.toRadians(-70.0))
            .build()

        val parkLeftTrajectory = robot.drive.trajectorySequenceBuilder(trajectory5.end())
            .lineTo(Vector2d(46.0, -5.5))
            .waitSeconds(instantTime)
            .splineToSplineHeading(Pose2d(53.0, 5.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineTo(Vector2d(53.0, 20.0), Math.toRadians(90.0))
            .build()

        val parkCenterTrajectory = robot.drive.trajectorySequenceBuilder(trajectory5.end())
            .lineTo(Vector2d(46.0, -5.5))
            .waitSeconds(instantTime)
            .splineToSplineHeading(Pose2d(53.0, 2.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .build()

        val parkRightTrajectory = robot.drive.trajectorySequenceBuilder(trajectory5.end())
            .lineTo(Vector2d(46.0, -4.5))
            .waitSeconds(instantTime)
            .splineToSplineHeading(Pose2d(53.0, -8.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .build()

        // The following loop replaces waitForStart().
        while (!isStarted && !isStopRequested) {
            val newParkingLocation = parkingLocationAnalyzer.parkingLocation ?: continue

            parkingLocation = newParkingLocation

            telemetry.addData("Parking Location", parkingLocation)
            telemetry.update()
        }

        parkingLocationAnalyzer.stop()

        CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    FollowTrajectorySequenceCommand(robot.drive, trajectory1),
                    LiftPositionCommand(robot.lift, LiftPositionCommand.Position.GROUND),
                    GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE),
                    WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH))
                ),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.LOWER_THROUGH_HIGH),
                GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN),
                ParallelCommandGroup(
                    FollowTrajectorySequenceCommand(robot.drive, trajectory2),
                    WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.CONE_STACK_5))
                ),
                WaitCommand(250).andThen(GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE)),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.MEDIUM),
                ParallelCommandGroup(
                    FollowTrajectoryCommand(robot.drive, trajectory3),
                    WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH))
                ),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.LOWER_THROUGH_HIGH),
                GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN),
                ParallelCommandGroup(
                    FollowTrajectorySequenceCommand(robot.drive, trajectory4),
                    WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.CONE_STACK_4))
                ),
                GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.MEDIUM),
                ParallelCommandGroup(
                    FollowTrajectoryCommand(robot.drive, trajectory5),
                    WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH))
                ),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.LOWER_THROUGH_HIGH),
                GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN),
                ParallelCommandGroup(
                    when (parkingLocation) {
                    ParkingLocation.LEFT -> FollowTrajectorySequenceCommand(robot.drive, parkLeftTrajectory)
                    ParkingLocation.CENTER -> FollowTrajectorySequenceCommand(robot.drive, parkCenterTrajectory)
                    ParkingLocation.RIGHT -> FollowTrajectorySequenceCommand(robot.drive, parkRightTrajectory)
                    },
                    WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.ZERO))
                ),
                InstantCommand(this::requestOpModeStop)
            )
        )

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run()
            robot.drive.update()
            robot.lift.update()

            telemetry.addData("Pose", robot.drive.poseEstimate)
            telemetry.update()
        }
    }
}
