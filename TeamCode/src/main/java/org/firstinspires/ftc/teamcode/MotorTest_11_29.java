package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Test 11/1", group="Teleop")
public class ServoTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo grabberLeft = null;
    //private Servo grabberRight = null;

    @Override
    public void runOpMode() {
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        //grabberRight = hardwareMap.get(Servo.class, "grabberRight");

        grabberLeft.setDirection(Servo.Direction.FORWARD);
        //grabberRight.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("grabberLeft Position", "%.2f", grabberLeft.getPosition());
            //telemetry.addData("grabberRight Position", "%.2f", grabberRight.getPosition());
            telemetry.update();
            if(gamepad1.left_bumper && !gamepad1.right_bumper) {
                grabberLeft.setPosition(0);
                //grabberRight.setPosition(0);
            }
            else if(gamepad1.right_bumper && !gamepad1.left_bumper) {
                grabberLeft.setPosition(1.0);
                //grabberRight.setPosition(1.0);
            }
        }

    }
}
