package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.PivotSubsystem

class PivotPositionCommand : CommandBase {
    enum class Position(val power: Double) {
        CLOCK90(-0.5),
        COUNTER90(0.5)
    }

    var pivotSubsystem: PivotSubsystem
    var position: Position

    var timer: ElapsedTime? = null
    var pivotTime = 740

    constructor(pivotSubsystem: PivotSubsystem, position: Position) {
        this.pivotSubsystem = pivotSubsystem
        this.position = position
    }

    override fun execute() {
        if (timer != null) {
            return
        }

        timer = ElapsedTime()
        pivotSubsystem.pivot.power = position.power
    }

    override fun isFinished(): Boolean {
        return if (timer == null) {
            false
        } else {
            timer!!.milliseconds() >= pivotTime
        }
    }

    override fun end(interrupted: Boolean) {
        pivotSubsystem.pivot.power = 0.0
    }
}
