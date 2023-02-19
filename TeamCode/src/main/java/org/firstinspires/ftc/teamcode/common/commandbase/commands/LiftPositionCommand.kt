package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.CommandBase
// import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem

import kotlin.math.abs

class LiftPositionCommand : CommandBase {
    companion object {
        private const val startStackPosition = 700
        private const val stackConeOffset = 150
    }

    enum class Position(val tick: Int) {
        HIGH(4100),
        LOWER_THROUGH_HIGH(3800),
        MEDIUM(2980),
        LOW(1900),
        CONE_STACK_5(startStackPosition),
        CONE_STACK_4(startStackPosition - stackConeOffset),
        CONE_STACK_3(startStackPosition - stackConeOffset * 2),
        CONE_STACK_2(startStackPosition - stackConeOffset * 3),
        GROUND(100),
        ZERO(0)
    }

    var allowedError = 50

    var liftSubsystem: LiftSubsystem
    var position = Position.ZERO
    var positionTick = Position.ZERO.tick
    
    // var timer = ElapsedTime()
    // var timeout = 0.0

    constructor(liftSubsystem: LiftSubsystem, position: Position) {
        this.liftSubsystem = liftSubsystem
        this.position = position
        this.positionTick = position.tick
    }

    constructor(liftSubsystem: LiftSubsystem, position: Int) {
        this.liftSubsystem = liftSubsystem
        this.positionTick = position
    }

    override fun initialize() {
        liftSubsystem.controller.targetPosition = positionTick
    }

    override fun isFinished(): Boolean {
        return abs(liftSubsystem.controller.targetPosition - liftSubsystem.controller.currentPosition) < allowedError
    }
}
