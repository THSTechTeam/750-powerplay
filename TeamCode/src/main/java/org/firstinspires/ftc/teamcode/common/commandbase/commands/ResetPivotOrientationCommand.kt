package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.PivotSubsystem

class ResetPivotOrientationCommand(
    var pivotSubsystem: PivotSubsystem
) : CommandBase() {
    var timer: ElapsedTime? = null

    var pivotCounterTime = 400
    var pivotCounterPower = 1.0
    var pivotClockTime = 400
    var pivotClockPower = -0.5

    enum class State {
        COUNTER,
        CLOCK
    }

    var state = State.COUNTER

    override fun execute() {
        if (timer == null) {
            timer = ElapsedTime()
            pivotSubsystem.pivot.power = pivotCounterPower
        }

        if (timer!!.milliseconds() >= pivotCounterTime && state == State.COUNTER) {
            pivotSubsystem.pivot.power = 0.0
            pivotSubsystem.pivot.power = pivotClockPower
            state = State.CLOCK
        }

        if (timer!!.milliseconds() >= (pivotClockTime + pivotCounterTime) && state == State.CLOCK) {
            pivotSubsystem.pivot.power = 0.0
        }
    }

    override fun isFinished(): Boolean {
        return if (timer == null) {
            false
        } else {
            timer!!.milliseconds() >= pivotCounterTime + pivotClockTime + 50
        }
    }
}
