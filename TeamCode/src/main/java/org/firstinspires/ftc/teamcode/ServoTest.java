package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Test 11/1", group="Teleop")
public class ServoTest extends LinearOpMode {

    private static final double PIVOT_FRONT_POSITION = 1;
    private static final double PIVOT_BACK_POSITION = 0.25;

    private static final double GRABBER_OPEN_POSITION = 0;
    private static final double GRABBER_CLOSED_POSITION = 1.0;

    private ElapsedTime runtime = new ElapsedTime();

    private Servo servoPivot = null;
    private Servo grabberLeft = null;
    private Servo grabberRight = null;

    @Override
    public void runOpMode() {
        servoPivot = hardwareMap.get(Servo.class, "servoPivot");
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");

        servoPivot.setDirection(Servo.Direction.FORWARD);
        grabberLeft.setDirection(Servo.Direction.FORWARD);
        grabberRight.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("servoPivot Position", "%.2f", servoPivot.getPosition());
            telemetry.addData("grabberLeft Position", "%.2f", grabberLeft.getPosition());
            telemetry.addData("grabberRight Position", "%.2f", grabberRight.getPosition());
            //telemetry.addData("grabberLeft Position", "%.2f", grabberLeft.getPosition());
            //telemetry.addData("grabberRight Position", "%.2f", grabberRight.getPosition());
            telemetry.update();
            if (gamepad1.dpad_left && !gamepad1.dpad_right){
                servoPivot.setPosition(PIVOT_FRONT_POSITION);
            }
            // Pivot to back
            else if(gamepad1.dpad_right && !gamepad1.dpad_left) {
                servoPivot.setPosition(PIVOT_BACK_POSITION);
            }

            // Open grabber
            if(gamepad1.left_bumper && !gamepad1.right_bumper) {
                grabberLeft.setPosition(GRABBER_OPEN_POSITION);
                grabberRight.setPosition(GRABBER_OPEN_POSITION);
            }
            // Close grabber
            else if(gamepad1.right_bumper && !gamepad1.left_bumper) {
                grabberLeft.setPosition(GRABBER_CLOSED_POSITION);
                grabberRight.setPosition(GRABBER_CLOSED_POSITION);
            }
            /*if(gamepad1.left_bumper && !gamepad1.right_bumper) {
                grabberLeft.setPosition(0);
                //grabberRight.setPosition(0);
            }
            else if(gamepad1.right_bumper && !gamepad1.left_bumper) {
                grabberLeft.setPosition(1.0);
                //grabberRight.setPosition(1.0);
            }*/
        }

    }
}
