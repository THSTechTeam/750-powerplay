
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


//first attempt at actual autonomous mode

@Autonomous(name="Auto BlueCheck", group="Auto")
public class BlueCaroCheck extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;
    private DcMotor carouselWheel = null;

    public void DriveForward(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public void stopRobot()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public void turnLeft(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        rearLeft.setPower(power);
        rearRight.setPower(-power);
    }

    public void turnRight(double power)
    {
        turnLeft(-power);
    }

    public void turnAround(){
        turnRight(.5);
        sleep(1000);
    }

    public void carousel(double power){
        carouselWheel.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        carouselWheel = hardwareMap.get(DcMotor.class, "carouselWheel");


        // Making a shot in the dark guess that the right ones need to be reeversed.
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        carouselWheel.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();

        DriveForward(.6);
        sleep(400);
        turnRight(.85);
        sleep(400);
        DriveForward(.5);
        sleep(2000);
        stopRobot();
        carouselWheel.setPower(-1);
        sleep(2750);
        turnLeft(1);
        sleep(500);
        DriveForward(.6);
        sleep(350);
    }
}