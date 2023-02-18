package org.firstinspires.ftc.teamcode.common.commandbase.subsystem

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

class PivotSubsystem(hardwareMap: HardwareMap) : SubsystemBase() {
    var pivot: CRServo

    init {
        pivot = hardwareMap.get(CRServo::class.java, "servoPivot")
    }
}
