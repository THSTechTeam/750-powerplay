package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.acmerobotics.roadrunner.geometry.Pose2d

import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

class FollowTrajectorySequenceCommand(
    var drive: MecanumDrive,
    var trajectorySequence: TrajectorySequence
) : CommandBase() {
    override fun initialize() {
        drive.followTrajectorySequenceAsync(trajectorySequence)
    }

    // override fun end(interrupted: Boolean) {
    //     drive.poseEstimate = Pose2d()
    // }

    override fun isFinished(): Boolean {
        return !drive.isBusy()
    }
}
