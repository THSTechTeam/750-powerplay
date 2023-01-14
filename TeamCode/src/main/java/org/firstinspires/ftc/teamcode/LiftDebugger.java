package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="Lift Debugger", group="TeleOp")
public class LiftDebugger extends LinearOpMode {
    private DcMotor motorLift = null;

    public static double motorPower = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");

        waitForStart();

        while (opModeIsActive()) {
            motorLift.setPower(gamepad1.right_stick_y * motorPower);

            telemetry.addData("Motor Power", motorLift.getPower());
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            telemetry.update();
            idle();
        }
    }
}
