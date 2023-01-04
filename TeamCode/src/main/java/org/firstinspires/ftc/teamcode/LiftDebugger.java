package org.firtsinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="Lift Debugger", group="TeleOp")
public class LiftDebugger extends LinearOpMode {
    private DcMotor motorLift = null;

    public static int LIFT_LEVEL_0 = 0;
    public static int LIFT_LEVEL_1 = 1500;
    public static int LIFT_LEVEL_2 = 2300;
    public static int LIFT_LEVEL_3 = 3300;
    public static double LIFT_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Lift Level 0", LIFT_LEVEL_0);
            telemetry.addData("Lift Level 1", LIFT_LEVEL_1);
            telemetry.addData("Lift Level 2", LIFT_LEVEL_2);
            telemetry.addData("Lift Level 3", LIFT_LEVEL_3);
            telemetry.addData("Lift Speed", LIFT_SPEED);
            telemetry.addData("Current Lift Level", motorLift.getCurrentPosition());
            telemetry.update();
        }

        while (opModeIsActive()) {
            if (gamepad2.x) {
                motorLift.setTargetPosition(LIFT_LEVEL_1);
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setPower(LIFT_SPEED);
            } else if(gamepad2.y) {
                motorLift.setTargetPosition(LIFT_LEVEL_2);
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setPower(LIFT_SPEED);
            } else if(gamepad2.b) {
                motorLift.setTargetPosition(LIFT_LEVEL_3);
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setPower(LIFT_SPEED);
            } else if(gamepad2.a) {
                motorLift.setTargetPosition(LIFT_LEVEL_0);
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setPower(LIFT_SPEED);
            }

            telemetry.addData("Current Lift Level", motorLift.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}