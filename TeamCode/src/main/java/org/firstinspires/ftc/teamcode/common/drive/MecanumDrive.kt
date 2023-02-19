package org.firstinspires.ftc.teamcode.common.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kV

class MecanumDrive(hardwareMap: HardwareMap) : com.acmerobotics.roadrunner.drive.MecanumDrive(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, lateralMultiplier) {
    companion object {
        private val TRANSLATIONAL_PID = PIDCoefficients(2.0, 0.0, 0.0)
        private val HEADING_PID = PIDCoefficients(6.0, 0.0, 0.0)

        private const val lateralMultiplier = 1.578947368421053
    }

    private val frontLeft: DcMotorEx
    private val frontRight: DcMotorEx
    private val backRight: DcMotorEx
    private val backLeft: DcMotorEx
    private val motors: List<DcMotorEx>

    private var follower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                Pose2d(0.1, 0.1, Math.toRadians(1.0)), 0.1);
    private var trajectorySequenceRunner = TrajectorySequenceRunner(follower, HEADING_PID)

    private var VEL_CONSTRAINT: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        AngularVelocityConstraint(MAX_ANG_VEL),
        MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)))
    private var ACCEL_CONSTRAINT: TrajectoryAccelerationConstraint = ProfileAccelerationConstraint(MAX_ACCEL)

    private var batteryVoltageSensor: VoltageSensor

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

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        localizer = StandardTrackingWheelLocalizer(hardwareMap)
    }

    fun trajectoryBuilder(startPose: Pose2d = poseEstimate, reversed: Boolean = false): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d = poseEstimate): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT, MAX_ANG_VEL, MAX_ANG_ACCEL)
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build())
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    fun getLastError(): Pose2d {
        return trajectorySequenceRunner.lastPoseError
    }

    fun update() {
        updatePoseEstimate()
        val driveSignal = trajectorySequenceRunner.update(poseEstimate, poseVelocity)

        if (driveSignal != null) {
            setDriveSignal(driveSignal)
        }
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) {
            update()
        }
    }

    fun isBusy(): Boolean {
        return trajectorySequenceRunner.isBusy
    }

    fun setWeightedDrivePower(drivePower: Pose2d, botHeading: Double) {
        val rotatedInput = Vector2d(drivePower.x, drivePower.y).rotated(botHeading)
        setWeightedDrivePower(Pose2d(rotatedInput.x, rotatedInput.y, drivePower.heading))
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        val denominator = max(
            abs(drivePower.x) + abs(drivePower.y) + abs(drivePower.heading),
            1.0
        )

        drivePower.div(denominator)
        val powers = robotToWheelVelocities(drivePower, 1.0, 1.0)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    override fun setMotorPowers(frontLeft: Double, backLeft: Double, backRight: Double, frontRight: Double) {
        this.frontLeft.power = frontLeft
        this.backLeft.power = backLeft
        this.backRight.power = backRight
        this.frontRight.power = frontRight
    }

    fun getMotorPowers(): List<Double> {
        return listOf(frontLeft.power, frontRight.power, backRight.power, backLeft.power)
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(frontLeft.currentPosition.toDouble()),
            encoderTicksToInches(frontRight.currentPosition.toDouble()),
            encoderTicksToInches(backRight.currentPosition.toDouble()),
            encoderTicksToInches(backLeft.currentPosition.toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(
            encoderTicksToInches(frontLeft.velocity),
            encoderTicksToInches(frontRight.velocity),
            encoderTicksToInches(backRight.velocity),
            encoderTicksToInches(backLeft.velocity)
        )
    }

    // Required roadrunner overrides. Not used as this class uses a three tracking wheel localizer and does
    // not rely on the IMU.
    override val rawExternalHeading: Double
        get() = 0.0

    override fun getExternalHeadingVelocity(): Double {
        return 0.0
    }

    private fun robotToWheelVelocities(
        robotVelocity: Pose2d,
        trackWidth: Double = 1.0,
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
