package org.firstinspires.ftc.teamcode.common.commandbase.auto

import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand

import org.firstinspires.ftc.teamcode.common.commandbase.commands.PivotPositionCommand
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ResetPivotOrientationCommand
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.PivotSubsystem

class PivotResetIntoClock90Sequence(
    pivotSubsystem : PivotSubsystem
) : SequentialCommandGroup(
    ResetPivotOrientationCommand(pivotSubsystem),
    WaitCommand(100).andThen(PivotPositionCommand(pivotSubsystem, PivotPositionCommand.Position.CLOCK90))
)
