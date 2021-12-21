
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//autonomous mode for red side closest to carousel, place closest puzzle piece to carousel along seam
@Autonomous(name="Auto RedCheck", group="Auto")
public class RedCaroCheck extends LinearOpMode {

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

        DriveForward(.85);
        sleep(800);
        turnLeft(.3);
        sleep(700);
        DriveForward(1);
        sleep(460);
        turnLeft(.225);
        sleep(1220);
        DriveForward(.35);
        sleep(2490);
        stopRobot();
        carouselWheel.setPower(1);
        sleep(2750);
        DriveForward(-80);
        sleep(350);

       /* turnRight(.6);
        sleep(755);
        DriveForward(1);
        sleep(1900);
        /*turnLeft(.5);
        sleep(1000);
        turnRight(.5);
        sleep(250);
*/

    }
}