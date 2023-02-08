package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="One Cone Left + Park", group="Autonomous")
public class OneConeAutoLeft extends LinearOpMode {
    private static final double INSTANT_TIME = 1.0e-6;

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;
    private SampleMecanumDrive drive;

    private DcMotorEx motorLift;

    public static int LIFT_LEVEL_0 = 100;
    public static int LIFT_LEVEL_1 = 1900;
    public static int LIFT_LEVEL_2 = 2980;
    public static int LIFT_LEVEL_3 = 4200;

    private CRServo servoGrabber;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        motorLift = hardwareMap.get(DcMotorEx.class, "motorLift");
        servoGrabber = hardwareMap.get(CRServo.class, "servoGrabber");


        // The following loop replaces `waitForStart()`.
        while (!isStarted() && !isStopRequested()) {
            ParkingLocation newParkingLocation = parkingLocationAnalyzer.getParkingLocation();

            if (newParkingLocation != null) {
                parkingLocation = newParkingLocation;
            }

            telemetry.addData("Parking Location:", parkingLocation);
            telemetry.update();
        }
    }
}
