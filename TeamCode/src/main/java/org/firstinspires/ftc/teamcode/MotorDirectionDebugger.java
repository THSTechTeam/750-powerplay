package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *                                    The buttons are mapped to match the wheels spatially if you
 *                                    were to rotate the gamepad 45deg°. x/square is the front left
 *                    ________        and each button corresponds to the wheel as you go clockwise
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 *                  \________/
 *
 * Uncomment the @Disabled tag below to use this opmode.
 */
@TeleOp(group = "drive")
public class MotorDirectionDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 0.5;

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private List<DcMotor> mecanumMotors;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");

        mecanumMotors = Arrays.asList(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (!isStopRequested() && opModeIsActive()) {
            if(gamepad1.x) {
                motorFrontLeft.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Left");
            } else if(gamepad1.y) {
                motorFrontRight.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
            } else if(gamepad1.b) {
                motorBackRight.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Right");
            } else if(gamepad1.a) {
                motorBackLeft.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Left");
            } else {
                for (DcMotor motor : mecanumMotors) {
                    motor.setPower(0);
                }

                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
