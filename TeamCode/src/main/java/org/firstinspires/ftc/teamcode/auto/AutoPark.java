package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(name="AutonomousParking", group="Autonomous")
public class AutoPark extends LinearOpMode {
    public static double ONE_TILE_DISTANCE = 23; // in

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;

    private TrajectorySequence left_parking; 
    private TrajectorySequence center_parking;
    private TrajectorySequence right_parking;

    @Override
    public void runOpMode() throws InterruptedException {
        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);

        // Roadrunner initialization.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        // Trajectories.
        center_parking = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(ONE_TILE_DISTANCE + 15)
                .back(15)
                .build();

        left_parking = drive.trajectorySequenceBuilder(center_parking.end())
                .turn(Math.toRadians(90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(-90))
                .build();

        right_parking = drive.trajectorySequenceBuilder(center_parking.end())
                .turn(Math.toRadians(-90))
                .forward(ONE_TILE_DISTANCE)
                .turn(Math.toRadians(90))
                .build();

        // vvv The following loop replaces `waitForStart()`. vvv
        while (!isStarted() && !isStopRequested()) {
            ParkingLocation newParkingLocation = parkingLocationAnalyzer.getParkingLocation();

            if (newParkingLocation != null) {
                parkingLocation = newParkingLocation;
            }

            telemetry.addData("Parking Location", parkingLocation);
            telemetry.update();
        }
        // ^^^ End of `waitForStart()` replacement. ^^^

        if (parkingLocation == null) {
            parkingLocation = ParkingLocation.LEFT;
        }

        drive.followTrajectorySequence(left_parking);

        if (parkingLocation == ParkingLocation.CENTER) {
            drive.followTrajectorySequence(center_parking);
        } else if (parkingLocation == ParkingLocation.RIGHT) {
            drive.followTrajectorySequence(right_parking);
        }
    }
}
