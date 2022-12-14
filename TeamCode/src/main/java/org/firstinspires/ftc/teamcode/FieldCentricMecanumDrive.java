package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Field Centric Mecanum Drive", group="TeleOp")
public class FieldCentricMecanumDrive extends LinearOpMode {
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor motorLift = null;
    private CRServo grabberLeft = null;
    private CRServo grabberRight = null;
    private CRServo servoPivot = null;

    /** Change these values to modify motor/servo positions and speeds ****************************/

    public static int LIFT_LEVEL_0 = 0;
    public static int LIFT_LEVEL_1 = 1450;//2900;
    public static int LIFT_LEVEL_2 = 2300;//4600;
    public static int LIFT_LEVEL_3 = 3250;//6500;
    public static double LIFT_SPEED = 1;

    private static final double PIVOT_FRONT_POSITION = 1;
    private static final double PIVOT_BACK_POSITION = 0.25;

    private static final double GRABBER_POWER = 1;

    //private static final double GRABBER_OPEN_POSITION = 0;
    //private static final double GRABBER_CLOSED_POSITION = 1;

    /**********************************************************************************************/

    public static double lowPowerFactor = 0.35;
    public static double highPowerFactor = 0.65;

    private int currentLiftLevel = 0;
    private double motorPowerFactor = lowPowerFactor;

    private double getPowerFactor(double previousPowerFactor) {
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
       servoPivot = hardwareMap.get(CRServo.class, "servoPivot");
        grabberLeft = hardwareMap.get(CRServo.class, "grabberLeft");
        grabberRight = hardwareMap.get(CRServo.class, "grabberRight");

       servoPivot.setDirection(CRServo.Direction.FORWARD);
       grabberLeft.setDirection(CRServo.Direction.FORWARD);
       grabberRight.setDirection(CRServo.Direction.REVERSE);

        // reverse left side motors
       motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: Brandon Note - IMU Code broken on vertical hubs
        // retrieve imu from hardware map
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);

        // The following loop replaces `waitForStart()`.
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("ArmMotorPosition", motorLift.getCurrentPosition());
            telemetry.update();
        }
        // End of `waitForStart()` replacement.

        if (isStopRequested()) {
            return;
        }

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Currently at", " at %7d", motorLift.getCurrentPosition());
                telemetry.update();

            /** Lift Code *************************************************************************/

                if(gamepad2.x){
                    motorLift.setTargetPosition(LIFT_LEVEL_1);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(LIFT_SPEED);
                }
                else if(gamepad2.y){
                    motorLift.setTargetPosition(LIFT_LEVEL_2);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(LIFT_SPEED);
                }
                else if(gamepad2.b){
                    motorLift.setTargetPosition(LIFT_LEVEL_3);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(LIFT_SPEED);
                }
                else if(gamepad2.a){
                    motorLift.setTargetPosition(LIFT_LEVEL_0);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(LIFT_SPEED);
                }

                if(gamepad2.dpad_up && !gamepad2.dpad_down) {
                    motorLift.setPower(1);
                }
                else if(gamepad2.dpad_down && !gamepad2.dpad_up) {
                    motorLift.setPower(0);
                }

            /** Pivot Code ************************************************************************/

                // Pivot to front
                if (gamepad2.dpad_left && !gamepad2.dpad_right){
                    servoPivot.setPower(-0.7);
                    //servoPivot.setPosition(PIVOT_FRONT_POSITION);
                }
                // Pivot to back
                else if(gamepad2.dpad_right && !gamepad2.dpad_left){
                    servoPivot.setPower(0.7);
                    //servoPivot.setPosition(PIVOT_BACK_POSITION);
                }
                else {
                    servoPivot.setPower(0);
                }

           /** Grabber Code ***********************************************************************/

                // Open grabber
                if(gamepad2.left_bumper && !gamepad2.right_bumper) {
                    //  grabberLeft.setPosition(GRABBER_OPEN_POSITION);
                    //  grabberRight.setPosition(GRABBER_OPEN_POSITION);
                    grabberLeft.setPower(GRABBER_POWER);
                    grabberRight.setPower(GRABBER_POWER);
                }
                // Close grabber
                else if(gamepad2.right_bumper && !gamepad2.left_bumper) {
                    // grabberLeft.setPosition(GRABBER_CLOSED_POSITION);
                    // grabberRight.setPosition(GRABBER_CLOSED_POSITION);
                    grabberLeft.setPower(-1 * GRABBER_POWER);
                    grabberRight.setPower(-1 * GRABBER_POWER);
                } else {
                    grabberLeft.setPower(0);
                    grabberRight.setPower(0);
                }

                /*telemetry.addData("grabberLeft Position", "%.2f", grabberLeft.getPosition());
                telemetry.addData("grabberRight Position", "%.2f", grabberRight.getPosition());
                telemetry.update();

                // Open grabber
                if(gamepad1.left_bumper && !gamepad1.right_bumper) {
                    grabberLeft.setPosition(GRABBER_OPEN_POSITION);
                    grabberRight.setPosition(GRABBER_OPEN_POSITION);
                }
                // Close grabber
                else if(gamepad1.right_bumper && !gamepad1.left_bumper) {
                    grabberLeft.setPosition(GRABBER_CLOSED_POSITION);
                    grabberRight.setPosition(GRABBER_CLOSED_POSITION);
                }*/

            /** Drive Code ************************************************************************/

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
