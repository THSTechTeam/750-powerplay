package org.firstinspires.ftc.teamcode.opmode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.common.hardware.Robot
import org.firstinspires.ftc.teamcode.util.PIDController

import kotlin.math.abs

@TeleOp(name = "TeleOpMode", group = "TeleOp")
class TeleOpMode : LinearOpMode() {
    private lateinit var grabber: CRServo
    private lateinit var pivot: CRServo

    private lateinit var lift: PIDController
    private var liftKp = 0.003
    private var liftKi = 0.0
    private var liftKd = 0.001

    private var manualLiftIncrement = 60

    private var liftLevel0 = 100.0
    private var liftLevel1 = 1900.0
    private var liftLevel2 = 2980.0
    private var liftLevel3 = 4200.0
    private var liftPower = 1.0

    private var startStackPosition = 700.0
    private var stackConeOffset = 150.0

    private var pivotPower = 0.7

    // TODO: Need to check these values.
    private var grabberOpenPower = 0.0
    private var grabberClosePower = 0.7

    private var lowPowerFactor = 0.6
    private var highPowerFactor = 0.7

    private lateinit var robot: Robot

    override fun runOpMode() {
        lift = PIDController(
            liftKp, 
            liftKi, 
            liftKd,
            hardwareMap.get(DcMotorEx::class.java, "motorLift"),
            DcMotorSimple.Direction.REVERSE
            )
        lift.setMaxMotorPower(liftPower)

        pivot = hardwareMap.get(CRServo::class.java, "servoPivot")
        grabber = hardwareMap.get(CRServo::class.java, "servoGrabber")

        var motorPowerFactor = lowPowerFactor

        robot = Robot(hardwareMap)
        robot.startIMUThread(this)

        waitForStart()

        if (isStopRequested) {
            return
        }

        while (opModeIsActive()) {
            telemetry.addData("Currently at", " at %7d", lift.currentPosition)
            telemetry.update()
            lift.update()

            // Lift Controls
            if (gamepad2.x) {
                lift.targetPosition = liftLevel1
            } else if (gamepad2.y) {
                lift.targetPosition = liftLevel2
            } else if (gamepad2.b) {
                lift.targetPosition = liftLevel3
            } else if (gamepad2.a) {
                lift.targetPosition = liftLevel0
            } else if (gamepad2.dpad_up) {
                lift.targetPosition = startStackPosition
            } else if (gamepad2.dpad_left) {
                lift.targetPosition = startStackPosition - stackConeOffset
            } else if (gamepad2.dpad_down) {
                lift.targetPosition = startStackPosition - (stackConeOffset * 2)
            }

            // Manual lift control.
            lift.targetPosition = lift.targetPosition - (gamepad2.left_stick_y * manualLiftIncrement)

            if (gamepad2.dpad_right) {
                lift.resetEncoder()
            }

            // Pivot Controls
            if (abs(gamepad2.right_stick_x) > 0.1){
                pivot.power = -gamepad2.right_stick_x * pivotPower
            } else {
                pivot.power = 0.0
            }

            // Grabber Controls
            if(gamepad2.right_trigger > 0.5) {
                grabber.power = grabberClosePower
            } else {
                grabber.power = grabberOpenPower
            }

            if (gamepad1.a) {
                motorPowerFactor = lowPowerFactor
            } else if (gamepad1.b) {
                motorPowerFactor = highPowerFactor
            }

            robot.drive.setWeightedDrivePower(Pose2d(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            ).times(motorPowerFactor))

            idle()
        }
    }
}
