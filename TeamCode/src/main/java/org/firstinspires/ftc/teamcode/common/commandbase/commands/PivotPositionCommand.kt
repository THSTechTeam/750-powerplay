package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.CommandBase

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.PivotSubsystem

class PivotPositionCommand : CommandBase {
    enum class Position(val positionTick: Double) {
        FORWARD(0.0),
        COUNTER90(0.5),
        COUNTER180(1.0)
    }

    var pivotSubsystem: PivotSubsystem
    var position: Position
    private val tolerance = 0.05

    constructor(pivotSubsystem: PivotSubsystem, position: Position) {
        this.pivotSubsystem = pivotSubsystem
        this.position = position
    }

    override fun initialize() {
        pivotSubsystem.pivot.position = position.positionTick
    }

    override fun isFinished(): Boolean {
        return pivotSubsystem.pivot.position - position.positionTick < tolerance
    }
}
