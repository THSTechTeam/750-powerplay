package org.firstinspires.ftc.teamcode;/*
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class AutoCodeSimple extends LinearOpMode {
    //INTRODUCE VARIABLES HERE

    private static final double TILE_METER_LENGTH = 0.6;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;
    private double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    private void forwardOneTile(DcMotor[] driveMotors) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        final double drivePower = 0.2;

        for (DcMotor motor : driveMotors) {
            motor.setPower(drivePower);
        }

        sleep((long) (TILE_METER_LENGTH / drivePower * 1300)); // but y tho

        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }

        // Drive backwards slightly to clear the cone that we just pushed out of the way.
        for (DcMotor motor : driveMotors) {
            motor.setPower(-drivePower);
        }

        sleep((long) (TILE_METER_LENGTH / drivePower * 300));

        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }
    }


    private void strafeLeftOneTile(DcMotor[] mecanumMotors) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        final double drivePower = 0.50;

        mecanumMotors[0].setPower(-drivePower);
        mecanumMotors[1].setPower(drivePower);
        mecanumMotors[2].setPower(drivePower);
        mecanumMotors[3].setPower(-drivePower);

        sleep((long) (TILE_METER_LENGTH / drivePower * 1000));

        for (DcMotor motor : mecanumMotors) {
            motor.setPower(0);
        }
    }

    private void strafeRightOneTile(DcMotor[] mecanumMotors) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        final double drivePower = 0.50;

        mecanumMotors[0].setPower(drivePower);
        mecanumMotors[1].setPower(-drivePower);
        mecanumMotors[2].setPower(-drivePower);
        mecanumMotors[3].setPower(drivePower);

        sleep((long) (TILE_METER_LENGTH / drivePower * 1000));

        for (DcMotor motor : mecanumMotors) {
            motor.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        DcMotor[] mecanumMotors = {
                hardwareMap.get(DcMotor.class, "motorFrontLeft"),
                hardwareMap.get(DcMotor.class, "motorBackLeft"),
                hardwareMap.get(DcMotor.class, "motorFrontRight"),
                hardwareMap.get(DcMotor.class, "motorBackRight")
        };

        mecanumMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop.");
            telemetry.update();

            forwardOneTile(mecanumMotors);
        }

        //auto code here
        //code for going to location is at end of code, this is just simple read and then go to location
        if (tagOfInterest.id == LEFT) {
            // Drive to the Left Zone.
            forwardOneTile(mecanumMotors);
            strafeLeftOneTile(mecanumMotors);
        } else if (tagOfInterest.id == MIDDLE) {
            // Drive to the Center Zone.
            forwardOneTile(mecanumMotors);
        } else if (tagOfInterest.id == RIGHT) {
            // Drive to the Right Zone.
            forwardOneTile(mecanumMotors);
            strafeRightOneTile(mecanumMotors);
        }

    }

    private void tagToTelemetry(AprilTagDetection detection) {
        try {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        } catch (NullPointerException e) {
            telemetry.addLine("No tag ID available.");
        }
    }
}