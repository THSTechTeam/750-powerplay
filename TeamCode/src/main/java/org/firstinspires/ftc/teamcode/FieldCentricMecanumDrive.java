package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Field Centric Mecanum Drive", group="TeleOp")
public class FieldCentricMecanumDrive extends LinearOpMode {
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor motorLift = null;
    private Servo grabberLeft = null;
    private Servo grabberRight = null;
    private Servo servoPivot = null;

    private final double lowPowerFactor = 0.3;
    private final double highPowerFactor = 0.75;

    private double motorPowerFactor = lowPowerFactor;

    private double getPowerFactor(final double previousPowerFactor) {
       if (gamepad1.a) {
           return lowPowerFactor;
       } else if (gamepad1.b) {
           return highPowerFactor;
       } else {
           return previousPowerFactor;
       }
   }


    @Override
    public void runOpMode() throws InterruptedException {
       motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
       motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
       motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
       motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
       motorLift = hardwareMap.dcMotor.get("motorLift");
       servoPivot = hardwareMap.servo.get("servoPivot");
       grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
       grabberRight = hardwareMap.get(Servo.class, "grabberRight");

        grabberLeft.setDirection(Servo.Direction.FORWARD);
        grabberRight.setDirection(Servo.Direction.FORWARD);

        // reverse left side motors
       motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: Brandon Note - IMU Code broken on vertical hubs
        // retrieve imu from hardware map
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) {
            return;
        }


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Currently at", " at %7d",
                        motorLift.getCurrentPosition());
                telemetry.update();

                if (gamepad1.x && !gamepad1.y) {
                   // Display it for the driver.
                    motorLift.setTargetPosition(5000);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(0.5);
                } else if (gamepad1.y && !gamepad1.x) {
                    motorLift.setTargetPosition(0);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(0.5);
                }

                   if (gamepad1.dpad_left && !gamepad1.dpad_right){
                       servoPivot.setPosition(0);
                } else if(gamepad1.dpad_right && !gamepad1.dpad_left){
                       servoPivot.setPosition(1);
                }

                telemetry.addData("grabberLeft Position", "%.2f", grabberLeft.getPosition());
                telemetry.addData("grabberRight Position", "%.2f", grabberRight.getPosition());
                telemetry.update();
                if(gamepad1.left_bumper && !gamepad1.right_bumper) {
                    grabberLeft.setPosition(0);
                    grabberRight.setPosition(0);
                }
                else if(gamepad1.right_bumper && !gamepad1.left_bumper) {
                    grabberLeft.setPosition(1.0);
                    grabberRight.setPosition(1.0);
                }


            // 4097 driver station assignees controller to gamepad2 by default
            final double y = -gamepad1.left_stick_y; // reversed
            final double x = -(gamepad1.left_stick_x * 1.0); // imperfect strafing fix & reversed
            final double rx = gamepad1.right_stick_x;

            // TODO: Brandon Note - IMU Code broken on vertical hubs
//            final double botHeading = imu.getAngularOrientation().firstAngle;

            // x / y offsets
//            final double rotY = y * Math.cos(botHeading) + x * Math.sin(botHeading);
//            final double rotX = -y * Math.sin(botHeading) + x * Math.cos(botHeading);
            final double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double[] motorPowers = {
                    (y + x + rx) / denominator, // front left
                    (y - x + rx) / denominator, // back left
                    (y - x - rx) / denominator, // front right
                    (y + x - rx) / denominator, // back right
            };

            motorPowerFactor = getPowerFactor(motorPowerFactor);

            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] *= motorPowerFactor;
            }

            motorFrontLeft.setPower(motorPowers[0]);
            motorBackLeft.setPower(motorPowers[1]);
            motorFrontRight.setPower(motorPowers[2]);
            motorBackRight.setPower(motorPowers[3]);

            idle();

            }
            motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}