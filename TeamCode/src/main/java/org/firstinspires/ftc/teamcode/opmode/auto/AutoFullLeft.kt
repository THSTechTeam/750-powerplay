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

import org.firstinspires.ftc.teamcode.common.commandbase.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.GrabberStateCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand
import org.firstinspires.ftc.teamcode.common.hardware.Robot

@Autonomous(name = "Full Left", group = "auto")
class AutoFullLeft : LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        CommandScheduler.getInstance().reset()
        robot = Robot(hardwareMap, Robot.OpMode.AUTO)
        robot.drive.poseEstimate = Pose2d()

        val trajectory1 = robot.drive.trajectorySequenceBuilder()
            .lineTo(Vector2d(3.0, -6.0))
            .lineTo(Vector2d(55.0, 1.0))
            .lineTo(Vector2d(52.0, -6.0))
            .build()

        val trajectory2 = robot.drive.trajectorySequenceBuilder(trajectory1.end())
            .lineTo(Vector2d(55.0, -12.0))
            .build()

        val trajectory3 = robot.drive.trajectorySequenceBuilder(trajectory2.end())
            .lineTo(Vector2d(50.0, -13.0))
            .lineToSplineHeading(Pose2d(53.5, -1.5, Math.toRadians(90.0)))
            .build()

        val trajectory4 = robot.drive.trajectorySequenceBuilder(trajectory3.end())
            .lineTo(Vector2d(56.0, 18.0))
            .build()

        val trajectory5 = robot.drive.trajectorySequenceBuilder(trajectory4.end())
            .lineTo(Vector2d(50.0, 0.0))
            .lineToSplineHeading(Pose2d(52.0, -6.0, Math.toRadians(0.0)))
            .build()

        val trajectory6 = robot.drive.trajectorySequenceBuilder(trajectory5.end())
            .lineTo(Vector2d(55.0, -12.0))
            .build()

        val trajectoryParkCenter = robot.drive.trajectorySequenceBuilder(trajectory6.end())
            .lineTo(Vector2d(50.0, -13.0))
            .lineTo(Vector2d(53.0, 0.0))
            .build()

        waitForStart()

        CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    LiftPositionCommand(robot.lift, LiftPositionCommand.Position.GROUND),
                    GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE),
                    FollowTrajectorySequenceCommand(robot.drive, trajectory1),
                    WaitCommand(5000).andThen(LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH))
                ),
                FollowTrajectorySequenceCommand(robot.drive, trajectory2),
                WaitCommand(750).andThen(ParallelCommandGroup(
                    GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN),
                    FollowTrajectorySequenceCommand(robot.drive, trajectory3),
                )),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.CONE_STACK_5),
                FollowTrajectorySequenceCommand(robot.drive, trajectory4),
                GrabberStateCommand(robot.grabber, GrabberStateCommand.State.CLOSE),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.MEDIUM),
                FollowTrajectorySequenceCommand(robot.drive, trajectory5),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.HIGH),
                FollowTrajectorySequenceCommand(robot.drive, trajectory6),
                WaitCommand(750).andThen(GrabberStateCommand(robot.grabber, GrabberStateCommand.State.OPEN)),
                FollowTrajectorySequenceCommand(robot.drive, trajectoryParkCenter),
                LiftPositionCommand(robot.lift, LiftPositionCommand.Position.ZERO),
                InstantCommand(this::requestOpModeStop)
            )
        )

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run()
            robot.drive.update()
            robot.lift.update()
        }
    }
}
