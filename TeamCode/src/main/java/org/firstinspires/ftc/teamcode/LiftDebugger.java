package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
@TeleOp(name="Lift Debugger", group="TeleOp")
public class LiftDebugger extends LinearOpMode {
    private PIDFController liftController;
    private static int manualIncrement = 25;

    public static double KP = 0.003;
    public static double KI = 0.0;
    public static double KD = 0.001;
    public static double KF = 0.0;
    public static double MAX_LIFT_POWER = 1.0;

    public static int LIFT_LEVEL_0 = 100;
    public static int LIFT_LEVEL_1 = 1500;
    public static int LIFT_LEVEL_2 = 2400;
    public static int LIFT_LEVEL_3 = 3400;

    @Override
    public void runOpMode() throws InterruptedException {
        liftController = new PIDFController(
            KP,
            KI,
            KD,
            KF,
            537.7,
            hardwareMap.get(DcMotorEx.class, "motorLift"),
            DcMotorSimple.Direction.REVERSE
        );

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Motor position", liftController.getCurrentPosition());
            telemetry.update();
        }

        while (opModeIsActive()) {
            liftController.update();
            liftController.setConstants(KP, KI, KD, KF);
            liftController.setMaxMotorPower(MAX_LIFT_POWER);

            liftController.setTargetPosition(liftController.getTargetPosition() - (int) (manualIncrement * gamepad1.right_stick_y));

            // Preset lift positions.
            if (gamepad1.x) {
                liftController.setTargetPosition(LIFT_LEVEL_1);
            }
            else if (gamepad1.y) {
                liftController.setTargetPosition(LIFT_LEVEL_2);
            }
            else if (gamepad1.b) {
                liftController.setTargetPosition(LIFT_LEVEL_3);
            }
            else if (gamepad1.a) {
                liftController.setTargetPosition(LIFT_LEVEL_0);
            }

            if (gamepad1.dpad_right) {
                liftController.resetEncoder();
            }

            telemetry.addData("Motor Power", liftController.getPower());
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Target Position", liftController.getTargetPosition());
            telemetry.addData("Motor position", liftController.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
