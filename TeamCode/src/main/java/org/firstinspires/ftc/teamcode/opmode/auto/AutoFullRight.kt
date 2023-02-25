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
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PivotResetIntoClock90Sequence
import org.firstinspires.ftc.teamcode.common.commandbase.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.GrabberStateCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.PivotPositionCommand
import org.firstinspires.ftc.teamcode.common.hardware.Robot

@Autonomous(name = "Full Right", group = "auto")
class AutoFullRight : LinearOpMode() {
    override fun runOpMode() {
        CommandScheduler.getInstance().reset()

        val robot = Robot(hardwareMap, Robot.OpMode.AUTO)
        robot.drive.poseEstimate = Pose2d()

        val parkingLocationAnalyzer = ParkingLocationAnalyzer(hardwareMap)
        var parkingLocation = ParkingLocation.CENTER

        val trajectory1 = robot.drive.trajectorySequenceBuilder()
            .splineToConstantHeading(Vector2d(-3.0, -3.0), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(-45.0, -5.0), Math.toRadians(0.0))
            .splineToSplineHeading(Pose2d(-57.0, 6.0, Math.toRadians(60.0)), Math.toRadians(40.0))
            .build()

        val trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end(), true)
            .splineToSplineHeading(Pose2d(-55.0, -11.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-54.0, -34.0), Math.toRadians(-90.0))
            .build()

        val trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
            .splineToSplineHeading(Pose2d(-56.0, 3.5, Math.toRadians(70.0)), Math.toRadians(80.0))
            .build()

        val trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end(), true)
            .splineToSplineHeading(Pose2d(-57.0, -11.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-57.0, -35.0), Math.toRadians(-90.0))
            .build()

        val trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end())
            .splineToSplineHeading(Pose2d(-58.0, 2.50, Math.toRadians(70.0)), Math.toRadians(80.0))
            .build()

        val parkLeftTrajectory = robot.drive.trajectoryBuilder(trajectory5.end(), true)
            .splineToSplineHeading(Pose2d(-57.0, -11.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-55.0, -35.0), Math.toRadians-(90.0))
            .build()

        val parkCenterTrajectory = robot.drive.trajectoryBuilder(trajectory5.end(), true)
            .splineToSplineHeading(Pose2d(-57.0, -11.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
            .build()

        val parkRightTrajectory = robot.drive.trajectoryBuilder(trajectory5.end())
            .splineToSplineHeading(Pose2d(-57.0, 3.0, Math.toRadians(70.0)), Math.toRadians(90.0))
            .build()

        // The following loop replaces waitForStart().
        while (!isStarted && !isStopRequested) {
            telemetry.addData("Parking Location", parkingLocation)
            telemetry.update()

            val newParkingLocation = parkingLocationAnalyzer.parkingLocation ?: continue
            parkingLocation = newParkingLocation
        }

        parkingLocationAnalyzer.stop()

        CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    FollowTrajectorySequenceCommand(robot.drive, trajectory1),
                    LiftPositionCommand(robot.lift, LiftPositionCommand.Position.GROUND),
                    GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE),
                    WaitCommand(900).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH)),
                    WaitCommand(1500).andThen(PivotPositionCommand(robot.pivot, PivotPositionCommand.Position.COUNTER90)),
                ),
                WaitCommand(100).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.LOWER_THROUGH_HIGH)),
                WaitCommand(100).andThen(GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN)),
                ParallelCommandGroup(
                    FollowTrajectoryCommand(robot.drive, trajectory2),
                    WaitCommand(1000).andThen(ParallelCommandGroup(
                        LiftPositionCommand(robot.lift, LiftPositionCommand.Position.CONE_STACK_5),
                        PivotPositionCommand(robot.pivot, PivotPositionCommand.Position.COUNTER90),
                    )),
                ),
                GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE),
                ParallelCommandGroup(
                    LiftPositionCommand(robot.lift, LiftPositionCommand.Position.MEDIUM),
                    WaitCommand(700).andThen(ParallelCommandGroup(
                        PivotResetIntoClock90Sequence(robot.pivot),
                        ParallelCommandGroup(
                            FollowTrajectoryCommand(robot.drive, trajectory3),
                            WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH)),
                        )
                    ))
                ),
                // LiftPositionCommand(robot.lift, LiftPositionCommand.Position.LOWER_THROUGH_HIGH),
                WaitCommand(100).andThen(GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN)),
                // ParallelCommandGroup(
                //     FollowTrajectoryCommand(robot.drive, trajectory4),
                //     WaitCommand(1000).andThen(ParallelCommandGroup(
                //         LiftPositionCommand(robot.lift, LiftPositionCommand.Position.CONE_STACK_4),
                //         PivotPositionCommand(robot.pivot, PivotPositionCommand.Position.COUNTER90),
                //     )),
                // ),
                // GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE),
                // ParallelCommandGroup(
                //     LiftPositionCommand(robot.lift, LiftPositionCommand.Position.MEDIUM),
                //     WaitCommand(700).andThen(ParallelCommandGroup(
                //         PivotResetIntoClock90Sequence(robot.pivot),
                //         ParallelCommandGroup(
                //             FollowTrajectoryCommand(robot.drive, trajectory5),
                //             WaitCommand(1000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH)),
                //         )
                //     ))
                // ),
                // LiftPositionCommand(robot.lift, LiftPositionCommand.Position.LOWER_THROUGH_HIGH),
                // WaitCommand(100).andThen(GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN)),
                // ParallelCommandGroup(
                //     when (parkingLocation) {
                //         ParkingLocation.LEFT -> FollowTrajectoryCommand(robot.drive, parkLeftTrajectory)
                //         ParkingLocation.CENTER -> FollowTrajectoryCommand(robot.drive, parkCenterTrajectory)
                //         ParkingLocation.RIGHT -> FollowTrajectoryCommand(robot.drive, parkRightTrajectory)
                //     },
                //     WaitCommand(500).andThen(ParallelCommandGroup(
                //         LiftPositionCommand(robot.lift, LiftPositionCommand.Position.ZERO),
                //         PivotPositionCommand(robot.pivot, PivotPositionCommand.Position.CLOCK70),
                //     ))
                // ),
                WaitCommand(250).andThen(InstantCommand(this::requestOpModeStop))
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
