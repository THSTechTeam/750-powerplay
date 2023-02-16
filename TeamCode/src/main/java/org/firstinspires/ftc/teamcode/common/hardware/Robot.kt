package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive

class Robot {
    lateinit var drive: MecanumDrive

    private lateinit var imu: IMU
    private lateinit var imuThread: Thread
    var imuAngle = 0.0

    enum class OpMode {
        TELEOP,
        AUTO
    }

    var opMode = OpMode.TELEOP

    constructor(hardwareMap: HardwareMap, opMode: OpMode) {
        this.opMode = opMode
        drive = MecanumDrive(hardwareMap)

        imu = hardwareMap.get(IMU::class.java, "imu")
        val parameters = IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ))
        imu.initialize(parameters)
    }

    constructor(hardwareMap: HardwareMap) : this(hardwareMap, OpMode.TELEOP)

    fun startIMUThread(opMode: LinearOpMode) {
        @Synchronized
        fun getIMURobotHeading(): Double {
            return -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        }

        imuThread = Thread {
            while (opMode.opModeIsActive() && !opMode.isStopRequested) {
                imuAngle = getIMURobotHeading()
            }
        }
        imuThread.start()
    }
}
