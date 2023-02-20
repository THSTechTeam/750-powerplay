package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Servo Test", group="Teleop")
public class ServoTest extends LinearOpMode {
    private CRServo pivot;
    private CRServo grabber;

    public static int leftTime = 500;
    public static int rightTime = 500;
    public static double servoPower = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        pivot = hardwareMap.crservo.get("servoPivot");
        grabber = hardwareMap.crservo.get("servoGrabber");
        grabber.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_trigger > 0.5) {
                grabber.setPower(0);
            // Open grabber
            } else {
                grabber.setPower(0.7);
            }

            if (gamepad1.a) {
                pivot.setPower(servoPower);
                sleep(leftTime);
                pivot.setPower(0);
            } else if (gamepad1.b) {
                pivot.setPower(-servoPower);
                sleep(rightTime);
                pivot.setPower(0);
            }
        }
    }
}
