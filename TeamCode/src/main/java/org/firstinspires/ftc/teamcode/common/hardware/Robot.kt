package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.GrabberSubsystem
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem

class Robot {
    var asyncIMU: AsyncIMU? = null

    var drive: MecanumDrive
    var grabber: GrabberSubsystem
    var lift: LiftSubsystem

    enum class OpMode {
        TELEOP,
        AUTO
    }

    var opMode = OpMode.TELEOP

    constructor(hardwareMap: HardwareMap, opMode: OpMode) {
        this.opMode = opMode
        drive = MecanumDrive(hardwareMap)
        lift = LiftSubsystem(hardwareMap, opMode)
        grabber = GrabberSubsystem(hardwareMap)

        if (opMode == OpMode.TELEOP) {
            asyncIMU = AsyncIMU(hardwareMap)
        }
    }

    constructor(hardwareMap: HardwareMap) : this(hardwareMap, OpMode.TELEOP)

    fun startIMUThread(opMode: LinearOpMode) {
        asyncIMU?.startThread(opMode)
    }

    fun getIMUAngle(): Double {
        return asyncIMU?.angle ?: 0.0
    }
}
