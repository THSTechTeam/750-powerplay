package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    public static int LIFT_LEVEL_0 = 0;
    public static int LIFT_LEVEL_1 = 1500;
    public static int LIFT_LEVEL_2 = 2400;
    public static int LIFT_LEVEL_3 = 3400;
    public static double LIFT_SPEED = 1;

    private static final double PIVOT_POWER = 0.7;
    private static final double PIVOT_FRONT_POSITION = 1;
    private static final double PIVOT_BACK_POSITION = 0.25;

    private static final double GRABBER_POWER = 0.7;

    /**********************************************************************************************/

    public static double lowPowerFactor = 0.5;
    public static double highPowerFactor = 0.75;

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
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        servoPivot = hardwareMap.get(CRServo.class, "servoPivot");
        servoGrabber = hardwareMap.get(CRServo.class, "servoGrabber");

        servoGrabber.setDirection(CRServo.Direction.REVERSE);

        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

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

            if (gamepad2.dpad_right) {
                motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            /** Pivot Code ************************************************************************/

            if (Math.abs(gamepad2.right_stick_x) > 0.1){
                servoPivot.setPower(gamepad2.right_stick_x * PIVOT_POWER);
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

            motorPowerFactor = getPowerFactor(motorPowerFactor);

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);

            motorFrontLeft.setPower((x + y + rx) / denominator * motorPowerFactor);
            motorFrontRight.setPower((x - y - rx) / denominator * motorPowerFactor);
            motorBackLeft.setPower((x - y + rx) / denominator * motorPowerFactor);
            motorBackRight.setPower((x + y - rx) / denominator * motorPowerFactor);
            
            idle();
        }
    }
}
