package unused.code;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.BlockingDeque;

@Disabled
@TeleOp(name= "ElandOpMode", group = "TeleOp")
public class ElandOpMode extends LinearOpMode {
    private DcMotor lD, rD, bLD, bRD, gC;

    @Override
    public void runOpMode() throws InterruptedException {
        lD = hardwareMap.dcMotor.get("left_drive");
        rD = hardwareMap.dcMotor.get("right_drive");
        bLD = hardwareMap.dcMotor.get("back_left_drive");
        bRD = hardwareMap.dcMotor.get("back_right_drive");
        gC = hardwareMap.dcMotor.get("greenCircle");

        lD.setDirection(DcMotorSimple.Direction.REVERSE);
        bLD.setDirection(DcMotorSimple.Direction.REVERSE);

        double leftPower, rightPower;

        waitForStart();
        while (opModeIsActive()) {
            leftPower = (gamepad1.left_stick_y - gamepad1.left_stick_x);
            rightPower = (gamepad1.left_stick_y + gamepad1.left_stick_x);

            lD.setPower(leftPower);
            rD.setPower(rightPower);
            bLD.setPower(leftPower);
            bRD.setPower(rightPower);

            if (gamepad1.right_trigger > 0) {
                gC.setPower(0.75);
                gC.setPower(0);
            }
            else if (gamepad1.left_trigger > 0) {
                gC.setPower(-0.75);
                gC.setPower(0);
            }

            idle();
        }


    }

}
