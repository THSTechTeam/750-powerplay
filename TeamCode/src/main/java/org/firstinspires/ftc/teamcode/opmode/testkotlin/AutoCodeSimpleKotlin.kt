package org.firstinspires.ftc.teamcode.opmode.testkotlin

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.apriltag.AprilTagDetection
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.openftc.easyopencv.OpenCvCameraFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import java.lang.NullPointerException
import com.qualcomm.robotcore.eventloop.opmode.Disabled


/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
@Disabled
@Autonomous
class AutoCodeSimpleKotlin : LinearOpMode() {
    var camera: OpenCvCamera? = null
    var aprilTagDetectionPipeline: AprilTagDetectionPipeline? = null

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private val fx = 578.272
    private val fy = 578.272
    private val cx = 402.145
    private val cy = 221.506
    private val tagsize = 0.166

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/
    var LEFT = 1
    var MIDDLE = 2
    var RIGHT = 3
    var tagOfInterest: AprilTagDetection? = null
    private fun forwardOneTile(driveMotors: Array<DcMotor>) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        val drivePower = 0.2
        for (motor in driveMotors) {
            motor.power = drivePower
        }
        sleep((TILE_METER_LENGTH / drivePower * 1300).toLong()) // but y tho
        for (motor in driveMotors) {
            motor.power = 0.0
        }

        // Drive backwards slightly to clear the cone that we just pushed out of the way.
        for (motor in driveMotors) {
            motor.power = -drivePower
        }
        sleep((TILE_METER_LENGTH / drivePower * 300).toLong())
        for (motor in driveMotors) {
            motor.power = 0.0
        }
    }

    private fun strafeLeftOneTile(mecanumMotors: Array<DcMotor>) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        val drivePower = 0.50
        mecanumMotors[0].power = -drivePower
        mecanumMotors[1].power = drivePower
        mecanumMotors[2].power = drivePower
        mecanumMotors[3].power = -drivePower
        sleep((TILE_METER_LENGTH / drivePower * 1000).toLong())
        for (motor in mecanumMotors) {
            motor.power = 0.0
        }
    }

    private fun strafeRightOneTile(mecanumMotors: Array<DcMotor>) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        val drivePower = 0.50
        mecanumMotors[0].power = drivePower
        mecanumMotors[1].power = -drivePower
        mecanumMotors[2].power = -drivePower
        mecanumMotors[3].power = drivePower
        sleep((TILE_METER_LENGTH / drivePower * 1000).toLong())
        for (motor in mecanumMotors) {
            motor.power = 0.0
        }
    }

    override fun runOpMode() {
        val mecanumMotors = arrayOf(
                hardwareMap.get(DcMotor::class.java, "motorFrontLeft"),
                hardwareMap.get(DcMotor::class.java, "motorBackLeft"),
                hardwareMap.get(DcMotor::class.java, "motorFrontRight"),
                hardwareMap.get(DcMotor::class.java, "motorBackRight")
        )
        mecanumMotors[0].direction = DcMotorSimple.Direction.REVERSE
        mecanumMotors[2].direction = DcMotorSimple.Direction.REVERSE
        mecanumMotors[3].direction = DcMotorSimple.Direction.REVERSE
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        aprilTagDetectionPipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)
        (camera as OpenCvWebcam?)?.setPipeline(aprilTagDetectionPipeline)
        (camera as OpenCvWebcam?)?.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                (camera as OpenCvWebcam?)?.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })
        telemetry.msTransmissionInterval = 50


        //HARDWARE MAPPING HERE etc.


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */while (!isStarted && !isStopRequested) {
            val currentDetections = aprilTagDetectionPipeline!!.latestDetections
            if (currentDetections.size != 0) {
                var tagFound = false
                for (tag in currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag
                        tagFound = true
                        break
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:")
                    tagToTelemetry(tagOfInterest)
                } else {
                    telemetry.addLine("Don't see tag of interest :(")
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)")
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:")
                        tagToTelemetry(tagOfInterest)
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(")
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)")
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:")
                    tagToTelemetry(tagOfInterest)
                }
            }
            telemetry.update()
            sleep(20)
        }
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n")
            tagToTelemetry(tagOfInterest)
            telemetry.update()
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop.")
            telemetry.update()
            forwardOneTile(mecanumMotors)
        }

        //auto code here
        //code for going to location is at end of code, this is just simple read and then go to location
        if (tagOfInterest!!.id == LEFT) {
            // Drive to the Left Zone.
            forwardOneTile(mecanumMotors)
            strafeLeftOneTile(mecanumMotors)
        } else if (tagOfInterest!!.id == MIDDLE) {
            // Drive to the Center Zone.
            forwardOneTile(mecanumMotors)
        } else if (tagOfInterest!!.id == RIGHT) {
            // Drive to the Right Zone.
            forwardOneTile(mecanumMotors)
            strafeRightOneTile(mecanumMotors)
        }
    }

    private fun tagToTelemetry(detection: AprilTagDetection?) {
        try {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection!!.id))
        } catch (e: NullPointerException) {
            telemetry.addLine("No tag ID available.")
        }
    }

    companion object {
        //INTRODUCE VARIABLES HERE
        private const val TILE_METER_LENGTH = 0.6
        const val FEET_PER_METER = 3.28084
    }
}