package org.firstinspires.ftc.teamcode.common.drive

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.acmerobotics.roadrunner.geometry.Pose2d

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class MecanumDrive(hardwareMap: HardwareMap) {
    private val lateralMultiplier = 1.0

    private val frontLeft: DcMotorEx
    private val frontRight: DcMotorEx
    private val backRight: DcMotorEx
    private val backLeft: DcMotorEx
    private val motors: List<DcMotorEx>

    init {
        frontLeft = hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft")
        frontRight = hardwareMap.get(DcMotorEx::class.java, "motorFrontRight")
        backRight = hardwareMap.get(DcMotorEx::class.java, "motorBackRight")
        backLeft = hardwareMap.get(DcMotorEx::class.java, "motorBackLeft")

        motors = listOf(frontLeft, frontRight, backRight, backLeft)

        motors.forEach {
            val motorConfigurationType = it.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            it.motorType = motorConfigurationType
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
        frontRight.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setWeightedDrivePower(drivePower: Pose2d, botHeading: Double) {
        val denominator = max(
            abs(drivePower.x) + abs(drivePower.y) + abs(drivePower.heading),
            1.0
        )

        drivePower.div(denominator)
        setDrivePower(drivePower, botHeading)
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        val denominator = max(
            abs(drivePower.x) + abs(drivePower.y) + abs(drivePower.heading),
            1.0
        )

        drivePower.div(denominator)
        setDrivePower(drivePower)
    }

    fun setDrivePower(drivePower: Pose2d, botHeading: Double) {
        val adjustedX = -drivePower.y * sin(botHeading) + drivePower.x * cos(botHeading)
        val adjustedY = drivePower.y * cos(botHeading) + drivePower.x * sin(botHeading)

        val powers = robotToWheelVelocities(Pose2d(adjustedX, adjustedY, drivePower.heading), 1.0, 1.0)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    fun setDrivePower(drivePower: Pose2d) {
        val powers = robotToWheelVelocities(drivePower, 1.0, 1.0)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    fun setMotorPowers(frontLeft: Double, backLeft: Double, backRight: Double, frontRight: Double) {
        this.frontLeft.power = frontLeft
        this.backLeft.power = backLeft
        this.backRight.power = backRight
        this.frontRight.power = frontRight
    }

    private fun robotToWheelVelocities(
        robotVelocity: Pose2d,
        trackWidth: Double,
        wheelBase: Double = trackWidth
    ): List<Double> {
        val k = (trackWidth + wheelBase) / 2.0
        return listOf(
            robotVelocity.x - lateralMultiplier * robotVelocity.y - k * robotVelocity.heading,
            robotVelocity.x + lateralMultiplier * robotVelocity.y - k * robotVelocity.heading,
            robotVelocity.x - lateralMultiplier * robotVelocity.y + k * robotVelocity.heading,
            robotVelocity.x + lateralMultiplier * robotVelocity.y + k * robotVelocity.heading
        )
    }
}
