package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.DriveMode;
import org.firstinspires.ftc.teamcode.util.GamepadController;
import org.firstinspires.ftc.teamcode.util.GamepadButton;
import org.firstinspires.ftc.teamcode.util.MecanumDriveManager;

@TeleOp(name="Primary Drive", group="TeleOp")
public class PrimaryDrive extends LinearOpMode {
    private DcMotor motorLift = null;
    private CRServo grabberLeft = null;
    private CRServo grabberRight = null;
    private CRServo servoPivot = null;

    /** Change these values to modify motor/servo positions and speeds ****************************/

    private static final int LIFT_LEVEL_0 = 0;
    private static final int LIFT_LEVEL_1 = 1450;//2900;
    private static final int LIFT_LEVEL_2 = 2300;//4600;
    private static final int LIFT_LEVEL_3 = 3500;//6500;
    private static final double LIFT_SPEED = 1;

    private static final double PIVOT_FRONT_POSITION = 1;
    private static final double PIVOT_BACK_POSITION = 0.25;

    private static final double GRABBER_POWER = 1;

    //private static final double GRABBER_OPEN_POSITION = 0;
    //private static final double GRABBER_CLOSED_POSITION = 1;

    /**********************************************************************************************/


    private final double lowPowerFactor = 0.3;
    private final double highPowerFactor = 0.75;

    private int currentLiftLevel = 0;
    private double motorPowerFactor = lowPowerFactor;

    private GamepadController gamepadController1 = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        // Brandon's drive code init.
        MecanumDriveManager drive = new MecanumDriveManager(hardwareMap);
        drive.setMode(DriveMode.FIELD_CENTRIC);
        drive.flipY();

        motorLift = hardwareMap.dcMotor.get("motorLift");
        servoPivot = hardwareMap.get(CRServo.class, "servoPivot");
        grabberLeft = hardwareMap.get(CRServo.class, "grabberLeft");
        grabberRight = hardwareMap.get(CRServo.class, "grabberRight");

       servoPivot.setDirection(CRServo.Direction.FORWARD);
       grabberLeft.setDirection(CRServo.Direction.FORWARD);
       grabberRight.setDirection(CRServo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            telemetry.addData("Currently at", " at %7d", motorLift.getCurrentPosition());
            telemetry.update();
            gamepadController1.update(gamepad1);

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
            motorPowerFactor = getPowerFactor(motorPowerFactor);
        
            drive.setWeightedDrivePower(
                gamepadController1.getStick(GamepadButton.LEFT_STICK_X),
                gamepadController1.getStick(GamepadButton.LEFT_STICK_Y),
                gamepadController1.getStick(GamepadButton.RIGHT_STICK_X),
                motorPowerFactor
            );

            idle();
        }

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double getPowerFactor(double previousPowerFactor) {
        
    }
}