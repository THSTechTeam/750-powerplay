package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Direction TeleOp with Arm", group="Teleop")
public class BetterControlledDriveWArm extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft= null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft= null;
    private DcMotor rearRight = null;
    private DcMotor carouselWheel = null;
    private CRServo armMotorLeft = null;
    private CRServo armMotorRight = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        carouselWheel = hardwareMap.get(DcMotor.class, "carouselWheel");
        armMotorLeft = hardwareMap.get(CRServo.class, "armMotorLeft");
        armMotorRight = hardwareMap.get(CRServo.class, "armMotorRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        carouselWheel.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(CRServo.Direction.FORWARD);
        armMotorRight.setDirection(CRServo.Direction.REVERSE);

        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double carouselPower;
            double armMotorLeftPower;
            double armMotorRightPower;
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            carouselPower = gamepad1.right_trigger + -gamepad1.left_trigger;
            if(gamepad1.left_bumper && !gamepad1.right_bumper)  {
                armMotorLeftPower = 0.5;
                armMotorRightPower = 0.5;
            }
            else if(gamepad1.right_bumper && !gamepad1.left_bumper) {
                armMotorRightPower = -0.5;
                armMotorLeftPower = -0.5;
            }
            else {
                armMotorLeftPower = 0;
                armMotorRightPower = 0;
            }

            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            rearLeft.setPower(v3);
            rearRight.setPower(v4);
            carouselWheel.setPower(carouselPower);
            armMotorLeft.setPower(armMotorLeftPower);
            armMotorRight.setPower(armMotorRightPower);

        }

    }
}

