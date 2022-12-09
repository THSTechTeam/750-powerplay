package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Motor Test 11/29", group="Teleop")
public class MotorTest_11_29 extends LinearOpMode {

    private static final int LIFT_LEVEL_0 = 0;
    private static final int LIFT_LEVEL_1 = 1450;//2900;
    private static final int LIFT_LEVEL_2 = 2300;//4600;
    private static final int LIFT_LEVEL_3 = 3250;//6500;
    private static final double LIFT_SPEED = 1;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLift = null;

    @Override
    public void runOpMode() {
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");

        motorLift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //telemetry.addData("grabberLeft Position", "%.2f", motorLift.get);
            //telemetry.update();
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
        }
    }
}
