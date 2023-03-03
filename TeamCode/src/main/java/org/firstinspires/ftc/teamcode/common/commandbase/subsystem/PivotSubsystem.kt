package org.firstinspires.ftc.teamcode.common.commandbase.subsystem

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.HardwareMap

class PivotSubsystem(hardwareMap: HardwareMap) : SubsystemBase() {
    var pivot: Servo

    init {
        pivot = hardwareMap.get(Servo::class.java, "servoPivot")
    }
}
