package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Servo Test", group="Teleop")
public class ServoTest extends LinearOpMode {
    private Servo pivot;
    private CRServo grabber;

    public static double forwardPosition = 0.1;
    public static double position90 = 0.5;
    public static double position180 = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        pivot = hardwareMap.servo.get("servoPivot");
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
                pivot.setPosition(forwardPosition);
            } else if (gamepad1.b) {
                pivot.setPosition(position90);
            } else if (gamepad1.y) {
                pivot.setPosition(position180);
            }
        }
    }
}
