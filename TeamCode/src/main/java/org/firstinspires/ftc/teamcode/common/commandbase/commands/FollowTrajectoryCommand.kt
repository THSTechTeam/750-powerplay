package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory

import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive

class FollowTrajectoryCommand(
    var drive: MecanumDrive,
    var trajectory: Trajectory
) : CommandBase() {
    override fun initialize() {
        drive.followTrajectoryAsync(trajectory)
    }

    // override fun end(interrupted: Boolean) {
    //     drive.poseEstimate = Pose2d()
    // }

    override fun isFinished(): Boolean {
        return !drive.isBusy()
    }
}
