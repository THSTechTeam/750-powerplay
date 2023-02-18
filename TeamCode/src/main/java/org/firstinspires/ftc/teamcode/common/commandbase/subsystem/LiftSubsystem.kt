package org.firstinspires.ftc.teamcode.common.commandbase.subsystem

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.hardware.Robot.OpMode
import org.firstinspires.ftc.teamcode.common.hardware.PIDController

class LiftSubsystem : SubsystemBase {
    var controller: PIDController
    var kP = 0.003
    var kI = 0.0
    var kD = 0.001

    var opMode = OpMode.TELEOP

    constructor(hardwareMap: HardwareMap, opMode: OpMode) {
        this.opMode = opMode

        this.controller = PIDController(
            kP,
            kI,
            kD,
            hardwareMap.get(DcMotorEx::class.java, "motorLift"),
            DcMotorSimple.Direction.REVERSE
        )

        if (opMode == OpMode.AUTO) {
            controller.resetEncoder()
        }
    }

    constructor(hardwareMap: HardwareMap) : this(hardwareMap, OpMode.TELEOP)

    fun update() {
        controller.update()
    }
}
