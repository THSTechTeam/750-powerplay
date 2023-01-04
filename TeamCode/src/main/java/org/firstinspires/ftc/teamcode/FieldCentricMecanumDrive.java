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
    private CRServo servoGrabber = null;
    private CRServo servoPivot = null;

    /** Change these values to modify motor/servo positions and speeds ****************************/

    // These values are marked as public to allow the dashboard to display them for tuning. 
    public static int LIFT_LEVEL_0 = -1075;
    public static int LIFT_LEVEL_1 = 450;//2900;
    public static int LIFT_LEVEL_2 = 1300;//4600;
    public static int LIFT_LEVEL_3 = 2500;//6500;
    private static final double LIFT_SPEED = 1;

    private static final double PIVOT_POWER = 0.7;
    private static final double PIVOT_FRONT_POSITION = 1;
    private static final double PIVOT_BACK_POSITION = 0.25;

    private static final double GRABBER_POWER = 0.7;

    /**********************************************************************************************/

    private final double lowPowerFactor = 0.3;
    private final double highPowerFactor = 0.75;

    private int currentLiftLevel = 0;
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
        servoPivot = hardwareMap.get(CRServo.class, "servoPivot");
        servoGrabber = hardwareMap.get(CRServo.class, "servoGrabber");

       servoPivot.setDirection(CRServo.Direction.REVERSE);
       servoGrabber.setDirection(CRServo.Direction.REVERSE);
       motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

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
                if (gamepad2.left_bumper && !gamepad2.right_bumper){
                    servoPivot.setPower(PIVOT_POWER);
                }
                // Pivot to back
                else if(gamepad2.right_bumper && !gamepad2.left_bumper){
                    servoPivot.setPower(-PIVOT_POWER);
                }
                else {
                    servoPivot.setPower(0);
                }

           /** Grabber Code ***********************************************************************/

                // Close grabber
                if(gamepad2.right_trigger > 0.5) {
                    servoGrabber.setPower(0);
                // Open grabber
                } else {
                    servoGrabber.setPower(GRABBER_POWER);
                }

            /** Drive Code ************************************************************************/

            final double y = -gamepad1.left_stick_y; // reversed
            final double x = gamepad1.left_stick_x;
            final double rx = gamepad1.right_stick_x;
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
